"""prarob_interact package exports.

Heavy ROSA/OpenAI imports are loaded lazily so utility modules such as
``path_planning`` can be imported and tested without LLM dependencies.
"""

__all__ = ["get_llm", "create_agent"]


def __getattr__(name):
    if name == "get_llm":
        from prarob_interact.config import get_llm

        return get_llm
    if name == "create_agent":
        from prarob_interact.agent import create_agent

        return create_agent
    raise AttributeError(name)
