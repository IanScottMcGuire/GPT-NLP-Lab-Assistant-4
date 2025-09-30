import streamlit as st
from streamlit_chat import message  # from pip install streamlit-chat
from ask_questions import ask_question  # your backend function

# Page setup
st.set_page_config(page_title="ECEN ChatBot", page_icon="🤖", layout="wide")

# Initialize chat history (stored in session state so it persists)
if "messages" not in st.session_state:
    st.session_state["messages"] = []

# Title
st.title("💬 ECEN ChatBot")

# Display chat history
for i, msg in enumerate(st.session_state["messages"]):
    if msg["role"] == "user":
        message(msg["content"], is_user=True, key=f"user_{i}")
    else:
        message(msg["content"], is_user=False, key=f"bot_{i}")

# User input (chat box at the bottom)
if prompt := st.chat_input("Ask me something..."):
    # 1. Save user message
    st.session_state["messages"].append({"role": "user", "content": prompt})

    # 2. Get response from backend
    with st.spinner("Thinking..."):
        try:
            response = ask_question(prompt)  # 🔗 calls your ask_questions.py
        except Exception as e:
            response = f"Error: {e}"

    # 3. Save bot response
    st.session_state["messages"].append({"role": "bot", "content": response})

    # 4. Refresh UI so new messages appear
    st.rerun()

