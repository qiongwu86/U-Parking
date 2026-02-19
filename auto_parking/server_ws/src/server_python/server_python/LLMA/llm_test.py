import openai
api_key = "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"

class Llama3:
    """
    使用 openai==0.27.2 版本的库，通过调用远程 API (如 NVIDIA) 来使用 Llama 3 模型。
    """
    def __init__(self, api_key: str, model_id: str = "meta/llama-3.3-70b-instruct"):
        """
        初始化 API 客户端。

        Args:
            api_key (str): 你的 NVIDIA API Key。
            model_id (str): 要调用的模型名称。
        """
        # 1. 设置 API Key 和 Base URL (这是 openai==0.27.2 的旧版写法)
        openai.api_key = api_key
        openai.api_base = "https://integrate.api.nvidia.com/v1"  # 注意旧版是 api_base

        # 2. 设置模型名称
        # 注意：meta/llama-3.3-70b-instruct 这个名称不存在，需要使用正确的名称。
        # 根据NVIDIA文档，正确的名称通常是 'meta/llama3-70b-instruct'
        self.model_id = model_id

    def ask(self, prompt: str) -> str:
        """
        向 API 发送请求并获取模型的回答。

        Args:
            prompt (str): 你的问题或提示词。

        Returns:
            str: 模型生成的回答。
        """
        try:
            # 3. 使用旧版的 API 调用方式
            response = openai.ChatCompletion.create(
                model=self.model_id,
                messages=[{"role": "user", "content": prompt}],
                max_tokens=8000,
                temperature=0.0,
                top_p=1.0,
                stream=False
            )
            # 4. 使用旧版的字典访问方式获取内容
            return response["choices"][0]["message"]["content"]

        except openai.error.OpenAIError as e:
            # 捕获 OpenAI 相关的错误
            return f"API 调用出错: {e}"
        except Exception as e:
            # 捕获其他未知错误
            return f"发生未知错误: {e}"


# --- 使用示例 ---
if __name__ == "__main__":
    NVIDIA_API_KEY = "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"
    # 创建 Llama3 API 客户端实例
    llm = Llama3(api_key=NVIDIA_API_KEY)

   
    question = """你知道你上次输出了什么吗"""       # 无记忆
    reply = llm.ask(question)

    print(f"问题: {question}")
    print(f"回答: {reply}")