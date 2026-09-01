#!/usr/bin/env python3

import argparse
import json
import random
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset


class ObstacleLSTM(nn.Module):
    def __init__(
        self,
        input_size=4,
        hidden_size=64,
        num_layers=2,
        prediction_steps=30,
        output_size=2,
        dropout=0.1,
    ):
        super().__init__()

        self.prediction_steps = prediction_steps
        self.output_size = output_size

        self.lstm = nn.LSTM(
            input_size=input_size,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0,
        )

        self.output_layer = nn.Sequential(
            nn.Linear(hidden_size, hidden_size),
            nn.ReLU(),
            nn.Linear(
                hidden_size,
                prediction_steps * output_size,
            ),
        )

    def forward(self, inputs):
        _, (hidden_state, _) = self.lstm(inputs)

        final_hidden = hidden_state[-1]
        output = self.output_layer(final_hidden)

        return output.view(
            -1,
            self.prediction_steps,
            self.output_size,
        )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Train dynamic-obstacle LSTM predictor.'
    )

    parser.add_argument(
        'dataset_directory',
        help='Directory containing smoke_002_*.npz',
    )

    parser.add_argument(
        'model_directory',
        help='Directory used to save trained models',
    )

    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--batch-size', type=int, default=64)
    parser.add_argument('--learning-rate', type=float, default=0.001)
    parser.add_argument('--hidden-size', type=int, default=64)
    parser.add_argument('--num-layers', type=int, default=2)
    parser.add_argument('--dropout', type=float, default=0.1)
    parser.add_argument('--patience', type=int, default=10)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--num-workers', type=int, default=0)

    return parser.parse_args()


def set_random_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)

    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def load_npz(path):
    data = np.load(path)

    inputs = torch.from_numpy(
        data['inputs'].astype(np.float32)
    )

    targets = torch.from_numpy(
        data['targets'].astype(np.float32)
    )

    raw_targets = torch.from_numpy(
        data['raw_targets'].astype(np.float32)
    )

    return inputs, targets, raw_targets


def create_loader(
    inputs,
    targets,
    batch_size,
    shuffle,
    num_workers,
):
    dataset = TensorDataset(inputs, targets)

    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=num_workers,
        pin_memory=torch.cuda.is_available(),
    )


def run_epoch(
    model,
    data_loader,
    loss_function,
    device,
    optimizer=None,
):
    training = optimizer is not None

    if training:
        model.train()
    else:
        model.eval()

    total_loss = 0.0
    total_samples = 0

    context = (
        torch.enable_grad()
        if training
        else torch.no_grad()
    )

    with context:
        for inputs, targets in data_loader:
            inputs = inputs.to(
                device,
                non_blocking=True,
            )

            targets = targets.to(
                device,
                non_blocking=True,
            )

            if training:
                optimizer.zero_grad(set_to_none=True)

            predictions = model(inputs)
            loss = loss_function(predictions, targets)

            if training:
                loss.backward()

                torch.nn.utils.clip_grad_norm_(
                    model.parameters(),
                    max_norm=1.0,
                )

                optimizer.step()

            batch_size = inputs.shape[0]

            total_loss += loss.item() * batch_size
            total_samples += batch_size

    return total_loss / max(total_samples, 1)


def calculate_metrics(
    model,
    inputs,
    raw_targets,
    target_mean,
    target_std,
    device,
    batch_size=256,
):
    model.eval()

    predictions = []

    with torch.no_grad():
        for start_index in range(
            0,
            len(inputs),
            batch_size,
        ):
            batch = inputs[
                start_index:
                start_index + batch_size
            ].to(device)

            prediction = model(batch).cpu().numpy()
            predictions.append(prediction)

    normalized_predictions = np.concatenate(
        predictions,
        axis=0,
    )

    raw_predictions = (
        normalized_predictions
        * target_std.reshape(1, 1, 2)
        + target_mean.reshape(1, 1, 2)
    )

    raw_targets_numpy = raw_targets.numpy()

    errors = np.linalg.norm(
        raw_predictions - raw_targets_numpy,
        axis=2,
    )

    ade = float(errors.mean())
    fde = float(errors[:, -1].mean())

    one_second_index = min(9, errors.shape[1] - 1)
    two_second_index = min(19, errors.shape[1] - 1)
    three_second_index = min(29, errors.shape[1] - 1)

    metrics = {
        'ade_m': ade,
        'fde_m': fde,
        'error_1s_m': float(
            errors[:, one_second_index].mean()
        ),
        'error_2s_m': float(
            errors[:, two_second_index].mean()
        ),
        'error_3s_m': float(
            errors[:, three_second_index].mean()
        ),
    }

    return metrics


def main():
    args = parse_arguments()
    set_random_seed(args.seed)

    dataset_directory = Path(
        args.dataset_directory
    ).expanduser().resolve()

    model_directory = Path(
        args.model_directory
    ).expanduser().resolve()

    model_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    train_path = (
        dataset_directory / 'smoke_002_train.npz'
    )

    validation_path = (
        dataset_directory
        / 'smoke_002_validation.npz'
    )

    test_path = (
        dataset_directory / 'smoke_002_test.npz'
    )

    normalization_path = (
        dataset_directory / 'normalization.json'
    )

    required_paths = [
        train_path,
        validation_path,
        test_path,
        normalization_path,
    ]

    for path in required_paths:
        if not path.exists():
            raise FileNotFoundError(
                f'Required file does not exist: {path}'
            )

    train_inputs, train_targets, _ = load_npz(
        train_path
    )

    validation_inputs, validation_targets, _ = load_npz(
        validation_path
    )

    test_inputs, test_targets, test_raw_targets = load_npz(
        test_path
    )

    with normalization_path.open(
        encoding='utf-8',
    ) as json_file:
        normalization = json.load(json_file)

    target_mean = np.asarray(
        normalization['target_mean'],
        dtype=np.float32,
    )

    target_std = np.asarray(
        normalization['target_std'],
        dtype=np.float32,
    )

    train_loader = create_loader(
        train_inputs,
        train_targets,
        args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
    )

    validation_loader = create_loader(
        validation_inputs,
        validation_targets,
        args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
    )

    device = torch.device(
        'cuda'
        if torch.cuda.is_available()
        else 'cpu'
    )

    print(f'Device: {device}')
    print(f'Train samples: {len(train_inputs)}')
    print(f'Validation samples: {len(validation_inputs)}')
    print(f'Test samples: {len(test_inputs)}')

    model = ObstacleLSTM(
        input_size=4,
        hidden_size=args.hidden_size,
        num_layers=args.num_layers,
        prediction_steps=30,
        output_size=2,
        dropout=args.dropout,
    ).to(device)

    loss_function = nn.SmoothL1Loss()

    optimizer = torch.optim.Adam(
        model.parameters(),
        lr=args.learning_rate,
    )

    scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
        optimizer,
        mode='min',
        factor=0.5,
        patience=3,
        min_lr=1e-6,
    )

    best_validation_loss = float('inf')
    epochs_without_improvement = 0

    best_model_path = (
        model_directory
        / 'dynamic_obstacle_lstm_best.pt'
    )

    training_history = []

    for epoch in range(1, args.epochs + 1):
        epoch_start_time = time.perf_counter()

        train_loss = run_epoch(
            model,
            train_loader,
            loss_function,
            device,
            optimizer=optimizer,
        )

        validation_loss = run_epoch(
            model,
            validation_loader,
            loss_function,
            device,
            optimizer=None,
        )

        scheduler.step(validation_loss)

        learning_rate = optimizer.param_groups[0]['lr']
        elapsed = time.perf_counter() - epoch_start_time

        training_history.append({
            'epoch': epoch,
            'train_loss': train_loss,
            'validation_loss': validation_loss,
            'learning_rate': learning_rate,
            'elapsed_seconds': elapsed,
        })

        print(
            f'Epoch {epoch:03d} | '
            f'train={train_loss:.6f} | '
            f'validation={validation_loss:.6f} | '
            f'lr={learning_rate:.7f} | '
            f'time={elapsed:.2f}s'
        )

        if validation_loss < best_validation_loss:
            best_validation_loss = validation_loss
            epochs_without_improvement = 0

            torch.save({
                'model_state_dict': model.state_dict(),
                'input_size': 4,
                'hidden_size': args.hidden_size,
                'num_layers': args.num_layers,
                'prediction_steps': 30,
                'output_size': 2,
                'dropout': args.dropout,
                'best_validation_loss': best_validation_loss,
                'normalization': normalization,
            }, best_model_path)

            print(f'  Saved best model: {best_model_path}')
        else:
            epochs_without_improvement += 1

        if epochs_without_improvement >= args.patience:
            print(
                f'Early stopping after {epoch} epochs.'
            )
            break

    checkpoint = torch.load(
        best_model_path,
        map_location=device,
        weights_only=False,
    )

    model.load_state_dict(
        checkpoint['model_state_dict']
    )

    metrics = calculate_metrics(
        model,
        test_inputs,
        test_raw_targets,
        target_mean,
        target_std,
        device,
    )

    print()
    print('Test metrics')
    print(f'  ADE: {metrics["ade_m"]:.6f} m')
    print(f'  FDE: {metrics["fde_m"]:.6f} m')
    print(
        f'  1 second error: '
        f'{metrics["error_1s_m"]:.6f} m'
    )
    print(
        f'  2 second error: '
        f'{metrics["error_2s_m"]:.6f} m'
    )
    print(
        f'  3 second error: '
        f'{metrics["error_3s_m"]:.6f} m'
    )

    model = model.cpu().eval()

    example_input = torch.zeros(
        1,
        20,
        4,
        dtype=torch.float32,
    )

    traced_model = torch.jit.trace(
        model,
        example_input,
    )

    torchscript_path = (
        model_directory
        / 'dynamic_obstacle_lstm.ts'
    )

    traced_model.save(str(torchscript_path))

    history_path = (
        model_directory
        / 'training_history.json'
    )

    with history_path.open(
        'w',
        encoding='utf-8',
    ) as json_file:
        json.dump(
            training_history,
            json_file,
            indent=2,
        )

    metrics_path = (
        model_directory
        / 'test_metrics.json'
    )

    with metrics_path.open(
        'w',
        encoding='utf-8',
    ) as json_file:
        json.dump(
            metrics,
            json_file,
            indent=2,
        )

    model_metadata = {
        'model_type': 'LSTM',
        'input_size': 4,
        'hidden_size': args.hidden_size,
        'num_layers': args.num_layers,
        'history_steps': 20,
        'prediction_steps': 30,
        'sample_dt': 0.1,
        'output_size': 2,
        'input_features': [
            'relative_x',
            'relative_y',
            'vx',
            'vy',
        ],
        'output_features': [
            'future_relative_x',
            'future_relative_y',
        ],
        'purpose': (
            'Smoke model for validating the training '
            'and deployment pipeline.'
        ),
    }

    metadata_path = (
        model_directory
        / 'model_metadata.json'
    )

    with metadata_path.open(
        'w',
        encoding='utf-8',
    ) as json_file:
        json.dump(
            model_metadata,
            json_file,
            indent=2,
        )

    print()
    print(f'Best checkpoint: {best_model_path}')
    print(f'TorchScript model: {torchscript_path}')
    print(f'Test metrics: {metrics_path}')


if __name__ == '__main__':
    main()
