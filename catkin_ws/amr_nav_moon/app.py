import os
import json
import glob
import streamlit as st
from PIL import Image
import subprocess
import threading

st.set_page_config(layout="wide")
st.title("Moondream Visual Detection")

BASE_DIR = os.getcwd()
PROMPT_FILE = os.path.join(BASE_DIR, "prompt.json")
OUTPUT_FOLDER = os.path.join(BASE_DIR, "output_images")

# Function to start moondream_loop.py as a background thread
def start_detection_loop():
    def run_loop():
        subprocess.run(["python3", "moondream_loop.py"])
    thread = threading.Thread(target=run_loop, daemon=True)
    thread.start()

# Start detection loop only once when Streamlit app runs
if "loop_started" not in st.session_state:
    start_detection_loop()
    st.session_state.loop_started = True

# Load current prompt or default
def load_prompt():
    if os.path.exists(PROMPT_FILE):
        with open(PROMPT_FILE, "r") as f:
            data = json.load(f)
            return data.get("prompt", "black bag")
    return "black bag"

current_prompt = load_prompt()

prompt = st.text_input("Enter detection prompt", value=current_prompt)

if st.button("Update Prompt"):
    with open(PROMPT_FILE, "w") as f:
        json.dump({"prompt": prompt}, f)
    st.success(f"Prompt updated to: {prompt}")

st.markdown("### Latest Processed Images")

if not os.path.exists(OUTPUT_FOLDER):
    st.warning(f"No images yet... (folder {OUTPUT_FOLDER} not found)")
else:
    image_files = sorted(
        glob.glob(os.path.join(OUTPUT_FOLDER, "*.png")),
        key=os.path.getmtime,
        reverse=True
    )
    st.write(f"Found {len(image_files)} image(s) in {OUTPUT_FOLDER}")
    if len(image_files) == 0:
        st.warning("Images folder exists but no PNG files found.")
    else:
        for img_path in image_files[:5]:
            st.image(Image.open(img_path), caption=os.path.basename(img_path), use_column_width=True)
