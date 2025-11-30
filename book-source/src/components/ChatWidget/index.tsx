import React, { useState, useEffect } from 'react';
import axios from 'axios';
import './styles.css';

const API_URL = 'http://localhost:8000';

const ChatWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<any[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId] = useState(() => `session-${Date.now()}`);
  const [conversationId, setConversationId] = useState<string | null>(null);
  const [selectedText, setSelectedText] = useState('');
  const [showSelectionButton, setShowSelectionButton] = useState(false);
  const [selectionPosition, setSelectionPosition] = useState({ x: 0, y: 0 });

  useEffect(() => {
    console.log('✅ ChatWidget mounted');

    const handleTextSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        setSelectedText(text);
        const range = selection?.getRangeAt(0);
        const rect = range?.getBoundingClientRect();

        if (rect) {
          setSelectionPosition({
            x: rect.left + rect.width / 2,
            y: rect.top - 10
          });
          setShowSelectionButton(true);
        }
      } else {
        setShowSelectionButton(false);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('keyup', handleTextSelection);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('keyup', handleTextSelection);
    };
  }, []);

  const toggleChat = () => setIsOpen(!isOpen);

  const askAboutSelection = async () => {
    const question = `What is this: "${selectedText}"`;
    console.log('🎯 Asking about selection:', question);

    setShowSelectionButton(false);
    setIsOpen(true);

    const userMessage = { role: 'user', content: question };
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      console.log('📡 Sending to API...');
      const response = await axios.post(`${API_URL}/api/chat`, {
        query: question,
        session_id: sessionId,
        conversation_id: conversationId
      });

      console.log('✅ Response:', response.data);

      const assistantMessage = {
        role: 'assistant',
        content: response.data.message,
        sources: response.data.sources
      };

      setMessages(prev => [...prev, assistantMessage]);

      if (response.data.conversation_id) {
        setConversationId(response.data.conversation_id);
      }
    } catch (error: any) {
      console.error('❌ Error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, error occurred. Try again.',
        error: true
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    const query = input;
    setInput('');
    setIsLoading(true);

    try {
      const response = await axios.post(`${API_URL}/api/chat`, {
        query,
        session_id: sessionId,
        conversation_id: conversationId
      });

      const assistantMessage = {
        role: 'assistant',
        content: response.data.message,
        sources: response.data.sources
      };

      setMessages(prev => [...prev, assistantMessage]);

      if (response.data.conversation_id) {
        setConversationId(response.data.conversation_id);
      }
    } catch (error: any) {
      console.error('❌ Error:', error);
      setMessages(prev => [...prev, {
        role: 'assistant',
        content: 'Sorry, error occurred. Try again.',
        error: true
      }]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: any) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {showSelectionButton && (
        <div
          className="selection-ask-button"
          style={{
            position: 'fixed',
            left: `${selectionPosition.x}px`,
            top: `${selectionPosition.y}px`,
            transform: 'translate(-50%, -100%)',
          }}
          onMouseDown={(e) => {
            e.preventDefault();
            askAboutSelection();
          }}
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
            <path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm1 17h-2v-2h2v2zm2.07-7.75l-.9.92C13.45 12.9 13 13.5 13 15h-2v-.5c0-1.1.45-2.1 1.17-2.83l1.24-1.26c.37-.36.59-.86.59-1.41 0-1.1-.9-2-2-2s-2 .9-2 2H8c0-2.21 1.79-4 4-4s4 1.79 4 4c0 .88-.36 1.68-.93 2.25z" />
          </svg>
          Ask AI
        </div>
      )}

      <button className="chat-widget-button" onClick={toggleChat}>
        <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
        {!isOpen && messages.length > 0 && (
          <span className="chat-widget-badge">{messages.length}</span>
        )}
      </button>

      {isOpen && (
        <div className="chat-widget-window">
          <div className="chat-widget-header">
            <div>
              <h3>Ask about the Book</h3>
              <p>Physical AI & Robotics Assistant</p>
            </div>
            <button onClick={toggleChat} className="chat-widget-close">×</button>
          </div>

          <div className="chat-widget-messages">
            {messages.length === 0 && (
              <div className="chat-widget-welcome">
                <svg width="48" height="48" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M12 2a2 2 0 0 1 2 2v2a2 2 0 0 1 -2 2h0a2 2 0 0 1 -2 -2v-2a2 2 0 0 1 2 -2z" />
                  <path d="M4 8l4 4m0 -4l-4 4m12 -4l4 4m0 -4l-4 4" />
                  <path d="M9 21v-8a3 3 0 0 1 6 0v8" />
                </svg>
                <h4>Hello! I'm your AI assistant</h4>
                <p>Ask me anything or highlight text to learn more!</p>
                <div className="chat-widget-suggestions">
                  <button onClick={() => { setInput("What is ROS 2?"); setTimeout(sendMessage, 100); }}>
                    What is ROS 2?
                  </button>
                  <button onClick={() => { setInput("How do I install Gazebo?"); setTimeout(sendMessage, 100); }}>
                    How do I install Gazebo?
                  </button>
                </div>
              </div>
            )}

            {messages.map((msg: any, idx: number) => (
              <div key={idx} className={`chat-message chat-message-${msg.role}`}>
                <div className="chat-message-content">
                  {msg.content}
                  {msg.sources && msg.sources.length > 0 && (
                    <div className="chat-message-sources">
                      <strong>Sources:</strong>
                      {msg.sources.map((source: any, i: number) => (
                        <div key={i} className="chat-source-item">
                          📚 {source.chapter_name} - {source.lesson_title}
                        </div>
                      ))}
                    </div>
                  )}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className="chat-message chat-message-assistant">
                <div className="chat-message-content chat-loading">
                  <span></span><span></span><span></span>
                </div>
              </div>
            )}
          </div>

          <div className="chat-widget-input-area">
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              rows={1}
              disabled={isLoading}
            />
            <button
              onClick={sendMessage}
              disabled={!input.trim() || isLoading}
              className="chat-send-button"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </div>
        </div >
      )}
    </>
  );
};

export default ChatWidget;
