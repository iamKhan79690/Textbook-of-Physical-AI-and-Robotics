# Tasks: OpenAI Agent Builder Integration

**Input**: Design documents from `/specs/001-openai-agent-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL and not included in this task list. Add tests if explicitly requested.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `book-source/src/` (Docusaurus React components)
- **Backend**: `rag-server/` (FastAPI Python server)
- **Theme**: `book-source/src/theme/` (Docusaurus theme overrides)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create ChatInterface component directory structure in book-source/src/components/ChatInterface/
- [x] T002 [P] Create hooks directory in book-source/src/components/ChatInterface/hooks/
- [x] T003 [P] Verify RAG server dependencies in rag-server/requirements.txt (FastAPI, OpenAI SDK, psycopg2, qdrant-client)
- [x] T004 [P] Add OPENAI_AGENT_WORKFLOW_ID to rag-server/.env template documentation
- [x] T005 Create theme Root directory structure in book-source/src/theme/Root/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T006 Implement POST /chat endpoint in rag-server/main.py that accepts ChatRequest schema
- [x] T007 Implement RAG context retrieval in /chat endpoint (internal call to /search_book)
- [x] T008 Implement OpenAI Agent Builder API integration in /chat endpoint with workflow ID
- [x] T009 Implement error handling in /chat endpoint (RAG errors, Agent API errors, network failures)
- [x] T010 Implement conversation thread ID management in /chat endpoint (generate/retrieve thread IDs)
- [x] T011 [P] Create useRAGContext hook in book-source/src/components/ChatInterface/hooks/useRAGContext.js
- [x] T012 [P] Create useAgentChat hook in book-source/src/components/ChatInterface/hooks/useAgentChat.js
- [x] T013 [P] Create base ChatInterface component structure in book-source/src/components/ChatInterface/index.js

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask Questions About Book Content (Priority: P1) 🎯 MVP

**Goal**: Enable readers to ask questions and receive context-aware answers from the OpenAI Agent Builder Agent

**Independent Test**: Open any book page, ask "What is ROS 2?", verify agent responds with relevant answer that references book material

### Implementation for User Story 1

- [x] T014 [US1] Implement message state management in book-source/src/components/ChatInterface/index.js (ChatMessage entities)
- [x] T015 [US1] Implement user query submission in book-source/src/components/ChatInterface/index.js (UserQuery entity)
- [x] T016 [US1] Integrate useAgentChat hook to call /chat endpoint in book-source/src/components/ChatInterface/index.js
- [x] T017 [US1] Implement message display (user messages and agent responses) in book-source/src/components/ChatInterface/index.js
- [x] T018 [US1] Implement input field and submit button in book-source/src/components/ChatInterface/index.js
- [x] T019 [US1] Add query validation (minLength 1, maxLength 2000) in book-source/src/components/ChatInterface/index.js per data-model.md

**Checkpoint**: At this point, User Story 1 should be fully functional - users can ask questions and receive agent responses

---

## Phase 4: User Story 2 - Chat Interface Available on All Pages (Priority: P1) 🎯 MVP

**Goal**: Ensure chat interface appears and functions identically on all Docusaurus pages (homepage, chapter pages, documentation pages)

**Independent Test**: Navigate to homepage, chapter page, and documentation page - verify chat interface appears and works on all pages

### Implementation for User Story 2

- [x] T020 [US2] Create or modify Root theme component in book-source/src/theme/Root/index.js
- [x] T021 [US2] Import ChatInterface component in book-source/src/theme/Root/index.js
- [x] T022 [US2] Render ChatInterface in Root component to ensure it appears on all pages in book-source/src/theme/Root/index.js
- [ ] T023 [US2] Verify chat interface appears on homepage (test manually or with integration test)
- [ ] T024 [US2] Verify chat interface appears on chapter pages (test manually or with integration test)
- [ ] T025 [US2] Verify chat interface appears on documentation pages (test manually or with integration test)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - chat is functional and available on all pages

---

## Phase 5: User Story 3 - RAG-Enhanced Responses (Priority: P2)

**Goal**: Ensure agent responses incorporate relevant book content retrieved from RAG server

**Independent Test**: Ask a question about a concept in Chapter 2, verify response includes information from Chapter 2 content retrieved via RAG

### Implementation for User Story 3

- [x] T026 [US3] Implement RAG context retrieval in useRAGContext hook in book-source/src/components/ChatInterface/hooks/useRAGContext.js
- [x] T027 [US3] Integrate useRAGContext hook into ChatInterface component in book-source/src/components/ChatInterface/index.js (RAG handled by backend /chat endpoint)
- [x] T028 [US3] Ensure RAG context is passed to /chat endpoint before agent call in book-source/src/components/ChatInterface/hooks/useAgentChat.js (Backend handles RAG internally)
- [x] T029 [US3] Verify backend /chat endpoint combines RAG context with user query before calling Agent Builder in rag-server/main.py
- [x] T030 [US3] Add RAG context usage indicator in chat UI (show when context was used) in book-source/src/components/ChatInterface/index.js
- [x] T031 [US3] Handle RAG server failure gracefully (still call agent, note context unavailable) in book-source/src/components/ChatInterface/hooks/useRAGContext.js (Backend handles gracefully)

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should work - responses are enhanced with book content

---

## Phase 6: User Story 4 - Chat Interface Integration with ChatKit (Priority: P2)

**Goal**: Provide polished, user-friendly chat UI with proper message display, input handling, and visual feedback

**Independent Test**: Interact with chat interface, verify clean UI, loading indicators, message history, and proper formatting

### Implementation for User Story 4

- [x] T032 [US4] Create chat interface styles in book-source/src/components/ChatInterface/styles.module.css
- [ ] T033 [US4] Implement loading state indicator during query processing in book-source/src/components/ChatInterface/index.js
- [ ] T034 [US4] Implement message history display with scrolling in book-source/src/components/ChatInterface/index.js
- [ ] T035 [US4] Implement conversation context persistence using sessionStorage in book-source/src/components/ChatInterface/index.js (Conversation entity)
- [ ] T036 [US4] Implement clear conversation / new conversation functionality in book-source/src/components/ChatInterface/index.js (FR-010)
- [ ] T037 [US4] Add responsive design for mobile devices (320px+ screen width) in book-source/src/components/ChatInterface/styles.module.css (FR-008, SC-007)
- [ ] T038 [US4] Implement error message display with user-friendly messages in book-source/src/components/ChatInterface/index.js (FR-009)
- [ ] T039 [US4] Add retry mechanism for failed requests in book-source/src/components/ChatInterface/index.js
- [ ] T040 [US4] Implement request queuing to prevent duplicate submissions in book-source/src/components/ChatInterface/index.js
- [ ] T041 [US4] Add text overflow handling for long messages in book-source/src/components/ChatInterface/styles.module.css

**Checkpoint**: At this point, all user stories should be complete - full chat functionality with polished UI

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T042 [P] Add global chat styles to book-source/src/css/custom.css for theme consistency
- [x] T043 [P] Update CORS settings in rag-server/main.py to allow frontend origin (already configured)
- [x] T044 [P] Add environment variable configuration for RAG server URL in frontend (development vs production)
- [x] T045 [P] Add conversation thread ID persistence in sessionStorage (already in US4, verify implementation)
- [x] T046 [P] Implement performance monitoring for response times (log response_time_ms from /chat endpoint)
- [ ] T047 [P] Add analytics tracking for user queries (optional, session_id tracking) - Optional, skipped for MVP
- [x] T048 [P] Verify all error codes from contracts/chat-api.yaml are handled in frontend
- [x] T049 [P] Add rate limiting handling (503 responses with retry_after) in book-source/src/components/ChatInterface/index.js
- [ ] T050 [P] Run quickstart.md validation steps to verify complete integration
- [ ] T051 [P] Test on mobile devices (320px+ screen width) to verify SC-007 compliance
- [ ] T052 [P] Verify chat interface loads within 2 seconds (SC-005) and optimize if needed
- [ ] T053 [P] Verify 90% of queries respond within 5 seconds (SC-001) and optimize if needed

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User Stories 1 and 2 (P1) can proceed in parallel after Foundational
  - User Story 3 (P2) depends on User Story 1 (needs chat working first)
  - User Story 4 (P2) can proceed in parallel with User Story 3
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Independent of US1, but both needed for MVP
- **User Story 3 (P2)**: Depends on User Story 1 (needs working chat to add RAG enhancement)
- **User Story 4 (P2)**: Can start after User Story 1 (needs basic chat working, but UI improvements are independent)

### Within Each User Story

- Hooks before components that use them
- Core functionality before UI polish
- Error handling after core implementation
- Story complete before moving to next priority

### Parallel Opportunities

- **Phase 1**: T002, T003, T004 can run in parallel (different directories/files)
- **Phase 2**: T011, T012, T013 can run in parallel (different hook/component files)
- **Phase 3 & 4**: User Stories 1 and 2 can be worked on in parallel after Foundational (different components)
- **Phase 5 & 6**: User Stories 3 and 4 can be worked on in parallel (different aspects - RAG vs UI)
- **Phase 7**: All polish tasks marked [P] can run in parallel

---

## Parallel Example: User Story 1 & 2 (MVP)

```bash
# After Foundational phase completes, these can run in parallel:

# Developer A: User Story 1 (Core chat functionality)
T014: Implement message state management
T015: Implement user query submission
T016: Integrate useAgentChat hook
T017: Implement message display
T018: Implement input field
T019: Add query validation

# Developer B: User Story 2 (Global integration)
T020: Create/modify Root theme component
T021: Import ChatInterface
T022: Render ChatInterface in Root
T023-T025: Verify on all page types
```

---

## Parallel Example: User Story 3 & 4 (Enhancements)

```bash
# After User Story 1 completes, these can run in parallel:

# Developer A: User Story 3 (RAG integration)
T026: Implement RAG context retrieval hook
T027: Integrate useRAGContext
T028: Pass RAG context to /chat
T029: Verify backend combines context
T030: Add context usage indicator
T031: Handle RAG failures

# Developer B: User Story 4 (UI polish)
T032: Create chat styles
T033: Implement loading indicator
T034: Implement message history
T035: Implement sessionStorage persistence
T036: Add clear conversation
T037: Add responsive design
T038: Implement error display
T039: Add retry mechanism
T040: Implement request queuing
T041: Add text overflow handling
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Core chat functionality)
4. Complete Phase 4: User Story 2 (Global page integration)
5. **STOP and VALIDATE**: Test both stories independently
6. Deploy/demo if ready

**MVP delivers**: Functional chat interface available on all pages with basic Q&A capability

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Stories 1 & 2 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 3 → Test independently → Deploy/Demo (RAG-enhanced responses)
4. Add User Story 4 → Test independently → Deploy/Demo (Polished UI)
5. Add Polish phase → Final production-ready version
6. Each increment adds value without breaking previous functionality

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Core chat)
   - Developer B: User Story 2 (Global integration)
3. Once User Story 1 is done:
   - Developer A: User Story 3 (RAG integration)
   - Developer B: User Story 4 (UI polish)
4. All developers: Polish phase tasks in parallel

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- User Stories 1 and 2 are both P1 and together form the MVP
- User Story 3 enhances User Story 1 with RAG context
- User Story 4 enhances all stories with polished UI/UX

