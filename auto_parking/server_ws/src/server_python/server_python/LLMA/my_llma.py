from .prompt import *
from .llm_test import Llama3
import re
import json

api_key="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"
model_id="meta/llama-3.3-70b-instruct"


# 解析大模型数据，返回result
def parse_llm_output_robust(text):
    """
    更健壮的解析器，处理多种格式变体
    """
    result = {
        'target_slot_id': None,
        'target_slot_coord': None,
        'key_path': None
    }

    # 提取target_slot_id (支持数字)
    id_match = re.search(r'target_slot_id[:\s]*(\d+)', text, re.IGNORECASE)
    if id_match:
        result['target_slot_id'] = int(id_match.group(1))

    # 提取target_slot_coord (支持多种分隔符)
    coord_match = re.search(r'target_slot_coord[:\s]*\[([\d.,\s]+)\]', text, re.IGNORECASE)
    if coord_match:
        coord_str = coord_match.group(1)
        # 解析坐标
        coords = [float(x.strip()) for x in coord_str.split(',')]
        result['target_slot_coord'] = coords

    # 提取key_path (更灵活的正则)
    # 匹配类似 [[...],[...]] 的嵌套列表
    path_pattern = r'key_path[:\s]*(\[(?:\[[\d.,\s]+\][,\s]*)+\])'
    path_match = re.search(path_pattern, text, re.IGNORECASE | re.DOTALL)

    if path_match:
        try:
            # 尝试用json解析
            path_str = path_match.group(1)
            # 如果包含非标准的浮点格式（如9.4）,需要先标准化
            path_str = re.sub(r'(?<=\d)\s*,\s*', ',', path_str)  # 去除逗号周围的空格
            result['key_path'] = json.loads(path_str)
        except json.JSONDecodeError:
            # 如果json解析失败，手动解析
            nested = path_match.group(1)
            # 移除外层括号
            inner = nested[1:-1]
            # 分割每个点
            points = re.findall(r'\[([\d.,\s]+)\]', inner)
            result['key_path'] = [[float(x.strip()) for x in p.split(',')] for p in points]

    return result

def my_llama3(api_key, model_id, start_pose, freelot_id):
    llm = Llama3(api_key,model_id)

    input_data={
        "start_pose": start_pose,
        "freelot_id": freelot_id
    }

    start_pose = tuple(input_data['start_pose'])
    freelot_id = tuple(input_data['freelot_id'])

    # print(start_pose,freelot_id)
    #
    query = my_llama.format(start_pose=start_pose, freelot_id=freelot_id)
    # response = self.model.ask(prompt=query)
    try:
        reply = llm.ask(prompt=query)
        print("LLAMA-A reply:\n",reply)
        return reply
    except Exception as e:
        print(f"发生错误: {e}")
        print("\n请检查以下几点：")
        print("1. 你的 openai 库版本是否确实是 0.27.2？ (pip show openai)")
        print("2. 你的 NVIDIA API Key 是否有效？")
        print("3. 旧版 openai 库对 api_base 的支持可能不稳定，如果失败，这可能是原因。")


if __name__ == "__main__":
    # # 测试解析函数
    # test_cases = [
    #     """target_slot_id: 23
    # target_slot_coord: [26.6, 35.0]
    # key_path: [[9.4, 45.57],[15.4, 45.57],[15.4, 29.05],[26.6, 29.05]]""",
    #
    #     """target_slot_id: 26
    # target_slot_coord: [34.6, 35.0]
    # key_path: [[9.4, 45.57], [15.4, 45.57], [15.4, 29.05], [34.6, 29.05]]""",
    #
    #     """TARGET_SLOT_ID: 15
    # Target_Slot_Coord: [32.3, 39.9]
    # Key_Path: [[9.4,45.57],[32.3,45.57]]"""
    # ]
    #
    # for i, output in enumerate(test_cases, 1):
    #     print(f"\n=== 测试用例 {i} ===")
    #     parsed = parse_llm_output_robust(output)
    #     print(f"target_slot_id: {parsed['target_slot_id']}")
    #     print(f"target_slot_coord: {parsed['target_slot_coord']}")
    #     print(f"key_path: {parsed['key_path']}")

    start_pose = [9.40, 45.57, 0.0]
    freelot_id = [1, 3, 5, 6, 13, 14, 16, 17, 18, 22, 23, 27, 31, 34, 38, 39, 42, 55, 59]
    # 解析大模型
    reply = my_llama3(api_key, model_id, start_pose, freelot_id)
    parsed = parse_llm_output_robust(reply)
    print(parsed['target_slot_id'])
    print(parsed['target_slot_coord'])
    print(parsed['key_path'])