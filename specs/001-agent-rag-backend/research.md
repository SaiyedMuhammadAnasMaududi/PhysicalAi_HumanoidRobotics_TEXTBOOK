# Research: Agent-Based RAG Backend using OpenAI Agent SDK with LiteLLM Models

**Feature**: 001-agent-rag-backend
**Date**: 2025-12-17
**Input**: Implementation plan from `plan.md`

## Research Focus

This research addresses the key technical questions identified in the implementation plan for integrating OpenAI Agent SDK with LiteLLM models, using function_tool for retrieval, and managing sessions with SQLiteSession.

## Research Questions & Findings

### 1. OpenAI Agent SDK Integration with LitellmModel

**Question**: How to properly initialize and configure an agents.Agent with LitellmModel?

**Research**:
- OpenAI's Agent SDK allows creating agents with custom tools/functions
- LitellmModel provides model abstraction for various LLM providers
- Need to use the agents.Agent class with proper model configuration

**Decision**: Use agents.Agent with LitellmModel for model configuration and function_tool for retrieval integration

**Rationale**: The agents.Agent class with LitellmModel provides proper model abstraction and follows the example provided by the user

**Implementation Pattern**:
```python
from agents import Agent
from agents.extensions.models.litellm_model import LitellmModel

# Initialize the model
model = LitellmModel(model="gemini/gemini-2.0-flash", api_key=api_key)

# Create the agent with the model
agent = Agent(
    name="RAG Agent",
    instructions="You are a helpful assistant that answers questions using provided context from technical book content.",
    model=model,
    tools=[retrieval_tool]  # function_tool for retrieval
)
```

**Reference**: OpenAI Agent SDK documentation and user example

### 2. LiteLLM Model Integration

**Question**: How to configure LitellmModel for proper model integration with agents.Agent?

**Research**:
- LitellmModel supports multiple providers including Gemini
- Configuration requires model name and API key
- Can be configured via environment variables or programmatic API

**Decision**: Use LitellmModel(model=model_name, api_key=api_key) for model configuration

**Rationale**: Provides maximum flexibility for model experimentation and proper integration with the agent SDK

**Implementation Pattern**:
```python
from agents.extensions.models.litellm_model import LitellmModel

# Configure via environment variables
model = LitellmModel(
    model=os.getenv("LITELLM_MODEL", "gemini/gemini-2.0-flash"),
    api_key=os.getenv("GEMINI_API_KEY")
)
```

**Reference**: LiteLLM and Agent SDK documentation

### 3. Function Tool Strategy

**Question**: What's the optimal way to expose retrieval as a function_tool for the agent?

**Research**:
- function_tool decorator allows creating tools for the agent
- Need to wrap the existing retrieval pipeline from Specs 1-2
- Tool should accept query parameters and return relevant context

**Decision**: Create a function_tool that wraps the existing retrieval pipeline from Specs 1-2

**Rationale**: Maintains proper separation of concerns and allows the agent to decide when to retrieve context

**Implementation Pattern**:
```python
from agents import function_tool

@function_tool()
def retrieve_context(query: str) -> str:
    """Retrieve relevant context from technical book content"""
    # Call existing retrieval pipeline from Specs 1-2
    results = retrieval_pipeline.query(query)
    return format_results_for_agent(results)
```

**Reference**: Agent SDK tool integration patterns

### 4. Runner Execution and Session Management

**Question**: How to properly execute agent via Runner.run with proper session management?

**Research**:
- Runner.run executes the agent with proper session handling
- SQLiteSession provides persistent session storage
- Need to pass context and session to the runner

**Decision**: Use Runner.run(agent, input, context, session) with SQLiteSession

**Rationale**: Provides proper execution management and persistent session handling across requests

**Implementation Pattern**:
```python
from agents import Runner
from agents import SQLiteSession

# Initialize session
session = SQLiteSession("agent_sessions.db")

# Run the agent
result = await Runner.run(
    agent=agent,
    input=user_query,
    context=context,
    session=session
)
```

**Reference**: Agent SDK execution documentation

## Architecture Pattern: Tool-Augmented RAG with Sessions

Based on research, the optimal architecture combines:

1. **agents.Agent**: For agent management and execution
2. **LitellmModel**: For model abstraction and configuration
3. **function_tool**: For retrieval integration
4. **Runner.run**: For proper execution management
5. **SQLiteSession**: For persistent session management
6. **FastAPI Endpoint**: For RESTful API access

This pattern ensures proper separation of concerns while maintaining the ability to use advanced agent capabilities with session persistence.

## Integration Points

### With Specs 1-2 Retrieval Pipeline
- Import and use the RetrievalPipeline class from `backend/src/retrieval.py`
- Pass queries from agent to retrieval pipeline via function_tool
- Format retrieved results for agent consumption

### With Environment Configuration
- Use existing .env pattern for API keys
- Add new variables for agent-specific configuration
- Maintain consistency with existing setup

## Risk Assessment

### Potential Issues
1. **Rate Limits**: LLM API rate limits could impact performance
2. **Cost Management**: Agent usage may be more expensive than simple completions
3. **Response Time**: Agent processing may be slower than direct LLM calls
4. **Context Window**: Retrieved context must fit within model's context window
5. **Session Management**: SQLiteSession needs proper concurrency handling

### Mitigation Strategies
1. Implement retry logic with exponential backoff
2. Use appropriate model selection based on cost/performance tradeoffs
3. Optimize retrieval to return only most relevant chunks
4. Implement context summarization for large result sets
5. Use connection pooling for SQLiteSession concurrency

## Conclusion

The research confirms that the planned architecture is feasible and follows industry best practices. The combination of agents.Agent with LitellmModel, function_tool for retrieval, and SQLiteSession for persistence provides the best integration with the existing retrieval pipeline while maintaining flexibility for future enhancements.