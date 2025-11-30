# API Contracts

This directory contains API contract specifications for the OpenAI Agent Builder Integration feature.

## Files

### `chat-api.yaml`
OpenAPI 3.0 specification for the new `/chat` endpoint that:
- Accepts user queries
- Retrieves RAG context
- Calls OpenAI Agent Builder Agent
- Returns agent responses

**Endpoint**: `POST /chat`

**Implementation Location**: `rag-server/main.py` (new endpoint to be added)

### `rag-api-reference.yaml`
OpenAPI 3.0 reference for the existing RAG server endpoint.

**Endpoint**: `POST /search_book`

**Implementation Location**: `rag-server/main.py` (already implemented)

## Usage

These contracts define:
1. Request/response schemas
2. Error handling patterns
3. Status codes and error codes
4. Validation rules

Use these contracts to:
- Generate API client code
- Validate request/response formats
- Document API behavior
- Write integration tests

## Integration Flow

```
Frontend (React Component)
  ↓ POST /chat
Backend (rag-server/main.py)
  ↓ POST /search_book (internal call)
RAG Server
  ↓ Returns context chunks
Backend
  ↓ Calls OpenAI Agent Builder API
OpenAI Agent Builder
  ↓ Returns agent response
Backend
  ↓ Returns to frontend
Frontend (displays response)
```

## Error Handling

All endpoints follow consistent error response format:
```json
{
  "error": "Human-readable message",
  "code": "MACHINE_READABLE_CODE",
  "details": "Optional debugging details",
  "retry_after": 60  // Optional, for rate limiting
}
```

## Validation

- Query length: 1-2000 characters
- Conversation ID: Valid UUID format
- All required fields must be present

