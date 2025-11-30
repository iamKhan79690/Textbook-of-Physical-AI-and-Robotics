# Research Findings: OpenAI Agent Builder Integration

**Feature**: OpenAI Agent Builder Integration  
**Date**: 2025-01-27  
**Purpose**: Resolve technical clarifications and identify best practices for integration

## Research Tasks

### Task 1: OpenAI ChatKit GitHub Repository

**Question**: What is the OpenAI ChatKit GitHub repository URL and how is it integrated?

**Research Method**: Web search and documentation review

**Findings**:

**Decision**: Use OpenAI's official Agent Builder embed script approach rather than a separate ChatKit repository.

**Rationale**: 
- OpenAI provides an official embed script (`https://platform.openai.com/agent-builder/embed.js`) for integrating Agent Builder agents
- This is the recommended approach for embedding agents into web applications
- The embed script handles the chat UI and agent communication automatically
- Configuration is done via data attributes on the script tag (e.g., `data-agent-id`)

**Alternatives Considered**:
1. **Separate ChatKit Repository**: Searched for "OpenAI ChatKit GitHub" but found that OpenAI's current approach uses the embed script rather than a standalone ChatKit component
2. **Custom React Chat Component**: Building a custom chat UI from scratch - rejected because the embed script provides a polished, maintained solution
3. **Third-party Chat Libraries**: Using libraries like react-chatbot-kit - rejected because they don't integrate directly with OpenAI Agent Builder workflow IDs

**Implementation Approach**:
- Use OpenAI's embed script in Docusaurus `docusaurus.config.js`
- Alternatively, create a React wrapper component that loads the embed script dynamically
- The wrapper approach provides better React integration and allows for custom styling/positioning

**Recommendation**: Create a React component wrapper around the OpenAI embed script to:
1. Maintain React component architecture consistency
2. Allow for custom styling and positioning
3. Enable better integration with Docusaurus theming
4. Provide hooks for RAG context integration

---

### Task 2: OpenAI Agent Builder Workflow Integration with RAG Context

**Question**: How to pass RAG-retrieved context to OpenAI Agent Builder Agent workflow?

**Research Method**: Analysis of OpenAI Agent Builder API and RAG server architecture

**Findings**:

**Decision**: Implement a proxy/backend endpoint that:
1. Receives user query from frontend
2. Calls RAG server `/search_book` endpoint to retrieve context
3. Combines user query + RAG context
4. Calls OpenAI Agent Builder API with workflow ID and enriched context
5. Returns agent response to frontend

**Rationale**:
- OpenAI Agent Builder workflows accept input parameters
- RAG context should be passed as part of the workflow input
- Backend proxy ensures API keys and workflow IDs remain secure
- Allows for request/response transformation and error handling

**Alternatives Considered**:
1. **Frontend-only integration**: Frontend calls RAG server, then Agent Builder - rejected because it exposes API keys and workflow IDs to the client
2. **RAG server modification**: Add agent integration to existing RAG server - acceptable but creates coupling; separate proxy is cleaner
3. **Direct Agent Builder with context in prompt**: Pass RAG context as part of user message - possible but less structured than workflow input

**Implementation Approach**:
- Create a new FastAPI endpoint in `rag-server/main.py` (or separate service)
- Endpoint: `POST /chat` that:
  - Accepts: `{ "query": "user question", "conversation_id": "optional" }`
  - Calls RAG server `/search_book` internally
  - Calls OpenAI Agent Builder API with workflow ID
  - Returns: `{ "response": "agent answer", "conversation_id": "..." }`

---

### Task 3: Docusaurus Component Integration Pattern

**Question**: Best practice for integrating a global component (chat interface) across all Docusaurus pages?

**Research Method**: Docusaurus documentation and existing codebase review

**Findings**:

**Decision**: Use Docusaurus's `@theme/Root` component to wrap the entire application with the chat interface.

**Rationale**:
- Docusaurus provides `@theme/Root` for global components that should appear on all pages
- This is the recommended approach for site-wide UI elements
- Maintains consistency with Docusaurus architecture
- Allows the chat to persist across page navigation

**Implementation Approach**:
1. Create `src/theme/Root/index.js` (or modify if exists)
2. Import and render ChatInterface component
3. Use Docusaurus's `useDocusaurusContext` for any site-wide configuration
4. Position chat interface using CSS (fixed position, bottom-right corner typical)

**Alternatives Considered**:
1. **Layout component**: Using `@theme/Layout` - rejected because it's page-specific, not site-wide
2. **Plugin approach**: Creating a Docusaurus plugin - overkill for a single component
3. **Individual page integration**: Adding to each page manually - rejected because it violates FR-012 (must be on all pages automatically)

---

### Task 4: Conversation Context Management

**Question**: How to maintain conversation context across multiple user queries?

**Findings**:

**Decision**: Use OpenAI Agent Builder's built-in conversation management via thread IDs, supplemented with local session storage for UI state.

**Rationale**:
- OpenAI Agent Builder API supports conversation threads
- Thread IDs can be stored in browser sessionStorage
- This maintains context on the agent side while keeping UI state client-side
- Allows for conversation history display in the chat interface

**Implementation Approach**:
- Generate or retrieve conversation thread ID on first message
- Store thread ID in sessionStorage
- Pass thread ID with each request to maintain context
- Clear thread ID when user starts new conversation (FR-010)

---

### Task 5: Error Handling and Fallback Strategies

**Question**: Best practices for handling RAG server errors and Agent Builder API failures?

**Findings**:

**Decision**: Implement graceful degradation with clear user messaging:
1. If RAG server fails: Still call Agent Builder but note that book context is unavailable
2. If Agent Builder fails: Display user-friendly error with retry option
3. Network errors: Show connection error with retry
4. Rate limiting: Display rate limit message with suggested wait time

**Rationale**:
- FR-009 requires graceful error handling
- Users should understand what went wrong
- System should remain partially functional when possible
- Clear messaging improves user experience

**Implementation Approach**:
- Try-catch blocks around RAG and Agent Builder calls
- Error state management in React component
- User-friendly error messages (not technical details)
- Retry mechanisms for transient failures

---

## Summary of Resolved Clarifications

1. ✅ **ChatKit Repository**: Use OpenAI embed script with React wrapper component
2. ✅ **RAG Context Integration**: Backend proxy endpoint that combines RAG + Agent Builder
3. ✅ **Docusaurus Integration**: Use `@theme/Root` component for site-wide chat
4. ✅ **Conversation Context**: OpenAI thread IDs + sessionStorage
5. ✅ **Error Handling**: Graceful degradation with clear user messaging

All technical clarifications have been resolved. The implementation approach is now clear and ready for Phase 1 design artifacts.

