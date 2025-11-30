# Quick Start Guide: OpenAI Agent Builder Integration

**Feature**: OpenAI Agent Builder Integration  
**Date**: 2025-01-27  
**Purpose**: Get the chat interface up and running quickly

## Prerequisites

- Node.js 18+ and npm
- Python 3.12+ (for RAG server)
- OpenAI API key with Agent Builder access
- OpenAI Agent Builder workflow ID
- RAG server running (see `rag-server/` directory)

## Setup Steps

### 1. Configure Environment Variables

Create or update `.env` file in `rag-server/` directory:

```bash
# Existing RAG server config
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=your_neon_db_url

# New: Agent Builder config
OPENAI_AGENT_WORKFLOW_ID=your_workflow_id_here
```

### 2. Install Dependencies

**Frontend (book-source/)**:
```bash
cd book-source
npm install
```

**Backend (rag-server/)** - if not already installed:
```bash
cd rag-server
pip install -r requirements.txt
# Add OpenAI SDK if not present:
pip install openai>=1.12.0
```

### 3. Start RAG Server

```bash
cd rag-server
python main.py
```

Server should start on `http://localhost:8000`

Verify it's running:
```bash
curl http://localhost:8000/
# Should return: {"status": "RAG Server is running"}
```

### 4. Add Chat Endpoint to RAG Server

Add the `/chat` endpoint to `rag-server/main.py` (see implementation plan for code).

The endpoint should:
- Accept `POST /chat` with `{ "query": "...", "conversation_id": "..." }`
- Call `/search_book` internally to get RAG context
- Call OpenAI Agent Builder API with workflow ID
- Return `{ "response": "...", "conversation_id": "..." }`

### 5. Create Chat Component

Create `book-source/src/components/ChatInterface/`:
- `index.js` - Main chat component
- `styles.module.css` - Chat styling
- `hooks/useRAGContext.js` - RAG integration hook
- `hooks/useAgentChat.js` - Agent Builder integration hook

### 6. Integrate into Docusaurus

Create or modify `book-source/src/theme/Root/index.js`:

```javascript
import React from 'react';
import ChatInterface from '@site/src/components/ChatInterface';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatInterface />
    </>
  );
}
```

### 7. Configure Chat Component

In `ChatInterface/index.js`, set:
- RAG server URL: `http://localhost:8000` (or environment variable)
- Agent workflow ID: From `.env` or configuration

### 8. Start Development Server

```bash
cd book-source
npm start
```

Open `http://localhost:3000` and verify chat interface appears on all pages.

## Testing the Integration

### Test RAG Server
```bash
curl -X POST http://localhost:8000/search_book \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

### Test Chat Endpoint
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

### Test in Browser
1. Open any page on the Docusaurus site
2. Look for chat interface (typically bottom-right corner)
3. Type a question: "What is ROS 2?"
4. Verify response includes book content context

## Troubleshooting

### Chat interface doesn't appear
- Check browser console for errors
- Verify `Root/index.js` is correctly importing ChatInterface
- Ensure component is exported correctly

### RAG server connection fails
- Verify RAG server is running on correct port
- Check CORS settings in `rag-server/main.py`
- Verify environment variables are set

### Agent Builder API errors
- Verify `OPENAI_AGENT_WORKFLOW_ID` is correct
- Check OpenAI API key has Agent Builder access
- Review API rate limits

### No context in responses
- Verify RAG server `/search_book` endpoint works
- Check that book content has been ingested (run `rag-server/ingest.py`)
- Verify RAG context is being passed to Agent Builder workflow

## Next Steps

1. Customize chat UI styling to match book theme
2. Add conversation history persistence (optional)
3. Implement error retry mechanisms
4. Add analytics tracking (optional)
5. Optimize for mobile devices

## Configuration Options

### Chat Position
Modify `ChatInterface/styles.module.css` to change position:
- Bottom-right (default)
- Bottom-left
- Top-right
- Floating button that expands

### RAG Context Limit
Configure how many chunks to retrieve:
- Default: 4 chunks (as per existing RAG server)
- Adjustable in `/chat` endpoint implementation

### Response Timeout
Set timeout for Agent Builder API calls:
- Recommended: 30 seconds
- Configurable in backend endpoint

## Production Deployment

1. Set production RAG server URL in environment variables
2. Use production OpenAI API keys (secure storage)
3. Enable CORS for production domain
4. Add error logging and monitoring
5. Test on mobile devices
6. Verify all pages have chat interface

## Support

For issues or questions:
- Check `research.md` for technical decisions
- Review `data-model.md` for data structures
- See `contracts/` for API specifications
- Refer to implementation plan for architecture details

