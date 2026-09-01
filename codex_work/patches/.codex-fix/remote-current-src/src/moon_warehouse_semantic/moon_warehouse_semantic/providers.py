"""Cloud model providers for the semantic parsing module."""

from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
from typing import Dict, Optional

import requests


class ProviderError(RuntimeError):
    """Raised when a model provider cannot return a usable result."""


@dataclass(frozen=True)
class SolverResult:
    """Structured variables returned by a model provider."""

    variables: Dict[str, int]
    provider: str
    model: str
    raw_text: str


class DeepSeekProvider:
    """Call DeepSeek through its OpenAI-compatible chat API."""

    def __init__(
        self,
        api_key: Optional[str] = None,
        base_url: str = 'https://api.deepseek.com',
        model: str = 'deepseek-v4-flash',
        timeout_sec: float = 20.0,
    ):
        self.api_key = (
            api_key
            or os.getenv('DEEPSEEK_API_KEY', '')
            or self._load_api_key_from_env_file()
        )
        self.endpoint = f"{base_url.rstrip('/')}/chat/completions"
        self.model = model
        self.timeout_sec = timeout_sec

    @staticmethod
    def _load_api_key_from_env_file() -> str:
        """Load the local competition key when a launcher did not export it."""
        env_path = Path.home() / '.reasonix' / '.env'
        try:
            lines = env_path.read_text(encoding='utf-8').splitlines()
        except OSError:
            return ''

        for raw_line in lines:
            line = raw_line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            name, value = line.split('=', 1)
            if name.strip().removeprefix('export ').strip() != 'DEEPSEEK_API_KEY':
                continue
            return value.strip().strip('"').strip("'")
        return ''

    @staticmethod
    def _extract_declared_variables(question: str) -> set[str]:
        """Extract variable names declared with phrases such as 数量为x."""
        matches = re.findall(
            r'(?:数量|数目|个数)?为\s*([A-Za-z][A-Za-z0-9_]*)',
            question,
        )
        return set(matches)

    @staticmethod
    def _has_positive_quantities(question: str) -> bool:
        """Return whether the question contains a positive counted quantity."""
        return re.search(
            r'(?<!\d)[1-9]\d*\s*'
            r'(?:个|件|只|条|台|盒|箱|瓶|本|张|块|套|组)',
            question,
        ) is not None

    def solve(self, question: str) -> SolverResult:
        """Solve declared variables in one question and return integers."""
        question = question.strip()

        if not question:
            raise ProviderError('Question is empty')

        if not self.api_key:
            raise ProviderError(
                'DEEPSEEK_API_KEY environment variable is not configured'
            )

        declared_variables = self._extract_declared_variables(question)
        declared_variables_hint = ''

        if declared_variables:
            declared_variables_hint = (
                '题目声明的变量名为：'
                + '、'.join(sorted(declared_variables))
                + '。variables必须包含且只能包含这些变量名。'
            )

        payload = {
            'model': self.model,
            'messages': [
                {
                    'role': 'system',
                    'content': (
                        '你是比赛题目求解器。'
                        '请根据题目完整计算其中明确声明的所有变量。'
                        '必须在内部完成需求汇总、容量换算、方程求解和结果核验。'
                        '必须采用以下赛题约定：当题目说明每个容器、商店、柜子或'
                        '其他资源单元可容纳N件物品，并列出多种类型时，'
                        '每种类型的资源单元只供应名称对应的同类物品。'
                        '先把所有人物对同一类型物品的需求相加，'
                        '特别注意并列主语：当题目写“甲、乙都需要N件”或'
                        '“甲、乙各需要N件”时，表示甲需要N件且乙也需要N件，'
                        '必须按人数分别累加，不能只把N计算一次；'
                        '三个或更多并列人物时也遵循同一规则。'
                        '再用该类型总需求除以单个资源单元容量N并向上取整；'
                        '即所需数量=(总需求+N-1)//N。'
                        '即使题目省略“每类资源只供应同类物品”这句话，'
                        '也必须按照上述赛题约定求解，不能以关系未说明为由拒绝。'
                        '如果题目要求计算容器、商店、柜子或其他资源数量，'
                        '结果必须满足全部需求，数量不能不足，必要时必须向上取整。'
                        '只计算题目中声明的变量，不执行颜色、区域、导航或抓取映射。'
                        '只输出一个JSON对象，不输出Markdown、推导过程或额外文字。'
                        'JSON必须包含variables、valid和error三个字段。'
                        'variables必须是变量名到非负整数的非空对象；'
                        'valid必须是布尔值；成功时valid为true且error为空字符串。'
                        '不得照抄格式说明中的任何数值，必须使用题目的实际计算结果。'
                        + declared_variables_hint
                    ),
                },
                {
                    'role': 'user',
                    'content': question,
                },
            ],
            'response_format': {'type': 'json_object'},
            'thinking': {'type': 'disabled'},
            'temperature': 0,
            'max_tokens': 512,
            'stream': False,
        }

        try:
            response = requests.post(
                self.endpoint,
                headers={
                    'Authorization': f'Bearer {self.api_key}',
                    'Content-Type': 'application/json',
                },
                json=payload,
                timeout=self.timeout_sec,
            )
        except requests.RequestException as exc:
            raise ProviderError(f'DeepSeek request failed: {exc}') from exc

        if response.status_code >= 400:
            detail = response.text.strip()[:300]
            raise ProviderError(
                f'DeepSeek returned HTTP {response.status_code}: {detail}'
            )

        try:
            api_data = response.json()
            raw_text = api_data['choices'][0]['message']['content']
        except (ValueError, KeyError, IndexError, TypeError) as exc:
            raise ProviderError(
                'DeepSeek response does not contain message content'
            ) from exc

        if not isinstance(raw_text, str) or not raw_text.strip():
            raise ProviderError('DeepSeek returned empty model content')

        try:
            result_data = json.loads(raw_text)
        except json.JSONDecodeError as exc:
            raise ProviderError(
                f'DeepSeek content is not valid JSON: {raw_text[:200]}'
            ) from exc

        if not isinstance(result_data, dict):
            raise ProviderError('DeepSeek JSON result must be an object')

        if result_data.get('valid') is not True:
            model_error = result_data.get(
                'error',
                'model did not confirm a valid result',
            )
            raise ProviderError(
                f'Model marked result invalid: {model_error}'
            )

        model_error = result_data.get('error')

        if not isinstance(model_error, str):
            raise ProviderError('Model result error field must be a string')

        if model_error.strip():
            raise ProviderError(
                f'Model returned an error with valid=true: {model_error}'
            )

        variables = result_data.get('variables')

        if not isinstance(variables, dict) or not variables:
            raise ProviderError('Model result has no variables object')

        normalized_variables = {}

        for name, value in variables.items():
            if not isinstance(name, str) or not name:
                raise ProviderError('Variable name is invalid')

            if isinstance(value, bool) or not isinstance(value, int):
                raise ProviderError(
                    f'Variable {name} must be an integer, got {value!r}'
                )

            if value < 0:
                raise ProviderError(
                    f'Variable {name} must not be negative'
                )

            normalized_variables[name] = value

        returned_variables = set(normalized_variables)

        if (
            declared_variables
            and returned_variables != declared_variables
        ):
            raise ProviderError(
                'Model variable names do not match the question: '
                f'expected={sorted(declared_variables)}, '
                f'got={sorted(returned_variables)}'
            )

        if (
            all(value == 0 for value in normalized_variables.values())
            and self._has_positive_quantities(question)
        ):
            raise ProviderError(
                'Model returned all-zero variables for a question '
                'containing positive quantities'
            )

        return SolverResult(
            variables=normalized_variables,
            provider='deepseek',
            model=self.model,
            raw_text=raw_text,
        )
