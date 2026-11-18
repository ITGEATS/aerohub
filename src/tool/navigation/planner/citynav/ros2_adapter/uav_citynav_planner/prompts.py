"""CityNavAgent 上游提示词的逐字复制（Tool 侧持有，2026-11-18 Tool 化改造）。

原居 llm_proxy 包（私有代理）；按"被复现工作的形式是 Tool"原则，prompt 属
导航 Tool 的算法语义，随 planner 迁移；Inference Function 只按 model_name
执行"帧+prompt → 文本"，不含任何导航语义（与 SPF prompt.py 同法）。

复制来源（upstream 子模块锁定提交 4b6e926c3325995483576523b18c9d0de5eb0710）：
  - src/llm/prompt_builder.py L304-334 route_planning_prompt_builder
  - src/llm/prompt_builder.py L282-301 landmark_caption_prompt_builder
  - utils/landmark_extraction.py L35-47 landmark 抽取 system/user 提示词
    （决策 7）

prompt 文本（含空白与示例）不得修改；tests/test_prompts.py 对 upstream 源
文件做逐字比对防漂移。
"""


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
