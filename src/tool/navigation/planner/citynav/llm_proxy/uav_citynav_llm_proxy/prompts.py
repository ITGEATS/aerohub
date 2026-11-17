"""CityNavAgent 上游提示词与响应解析的逐字复制。

复制来源（upstream 子模块锁定提交 4b6e926c3325995483576523b18c9d0de5eb0710）：
  - src/llm/prompt_builder.py L304-334 route_planning_prompt_builder
  - src/llm/prompt_builder.py L282-301 landmark_caption_prompt_builder
  - src/llm/prompt_builder.py L30-54   parse_viewpoint_response_v2
  - utils/landmark_extraction.py L35-47 landmark 抽取 system/user 提示词
    （决策 7 代决：Blocks 无 AirVLN-E 标注可消费，landmark 序列由 LLM 从指令
    抽取，即论文离线标注生成方法本身；README §2 称提示词取
    src/llm/prompt_builder.py，但该文件不含抽取提示词，逐字来源以本行为准）

复制原因：复现保真（迁移方案阶段 3）。许可证依据上游 README 徽章声明的 MIT
意图（CODE_LICENSE 文件断链缺口已记入过程记录，仅限组内非商业研究）。

prompt 文本（含空白与示例）与解析逻辑不得修改；tests/test_prompts.py 对
upstream 源文件做逐字比对，防止漂移。landmark_extraction_messages 是本包新增
薄封装，消息结构逐字取自 utils/landmark_extraction.py 的 create 调用，不改
提示词文本；其余三个函数为源段逐字复制。
"""
import json  # 上游 prompt_builder.py L3；parse_viewpoint_response_v2 依赖


def parse_viewpoint_response_v2(resp):
    cleaned_resp = resp.strip('`json\n').strip('`')
    cleaned_resp = cleaned_resp.replace("True", "true").replace("False", "false")

    json_resp = json.loads(cleaned_resp)
    reformat_resp = {}
    if "is_found" not in json_resp:
        print("Warning: LLM output missing 'is_found'")
        reformat_resp["is_found"] = False
    else:
        reformat_resp["is_found"] = json_resp["is_found"]

    for k in json_resp:
        if "slightly" in k and "left" in k:
            reformat_resp["slightly_left"] = json_resp[k]
        elif "slightly" in k and "right" in k:
            reformat_resp["slightly_right"] = json_resp[k]
        elif "left" in k:
            reformat_resp["left"] = json_resp[k]
        elif "right" in k:
            reformat_resp["right"] = json_resp[k]
        elif "front" in k:
            reformat_resp["front"] = json_resp[k]

    return reformat_resp


def landmark_caption_prompt_builder(scene_objects):
    if len(scene_objects) == 0:
        prompt = f"""
List the objects that appears in the images. Each object use no more than 5 words to describe.

Example output:
object1.object2.object3
"""
    else:
        prompt = f"""
List the objects that appears in the image from the list below:
{scene_objects}.

Example output:
object1.object2.object3

Your output:
    """

    return prompt


def route_planning_prompt_builder(navigation_instruction, total_subgoals, traversed_subgoals, next_subgoal):
    total_subgoals_str = ", ".join(total_subgoals)
    traversed_subgoals_str = ", ".join(traversed_subgoals)

    prompt = f"""
You are a drone and your task is navigating to the described target location!

Navigation instruction: {navigation_instruction}
You need to sequentially traverse all the following subgoals to reach the target: {total_subgoals_str}
The subgoals you have traversed are: {traversed_subgoals_str}
Your next navigation subgoal: {next_subgoal}

Your visual observation from different viewpoints are provide above.

Based on the instruction, next navigation subgoal and observation, you need to plan your next waypoint. 
There are two situations: 
    If you find the next subgoal, you should output the viewpoint that observes the subgoal in JSON format. E.g.:
{{
    "is_found": true,
    "slightly left": subgoal
}}
    If you don't find the next subgoal, you should select 3 objects you will probably go next from your observations in descending order of probability to find the subgoal. E.g.:
{{
    "is_found": false,
    "front": "object 1",
    "left": "object 2",
    "slightly left": "object 3"
}}
    """

    return prompt


LANDMARK_EXTRACTION_SYSTEM_PROMPT = 'You are a navigation aircraft, and now you need to navigate to a specified location according to a natural language instruction. You need to extract a landmark sequence from the instruction. The sequence order should be consistent with their appearance on the path. Your output should be in JSON format and must contain two fields: "Landmark sequence" and "Thought." "Landmark sequence" is your thinking result comprised of landmark phrases in the instruction. "Thought" is your thinking process.'


def landmark_extraction_user_prompt(instruction):
    return f'The instruction is <{instruction}>'


def landmark_extraction_messages(instruction):
    return [
        {"role": "system", "content": LANDMARK_EXTRACTION_SYSTEM_PROMPT},
        {"role": "user", "content": landmark_extraction_user_prompt(instruction)},
    ]
