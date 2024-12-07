from dotenv import load_dotenv
import os

load_dotenv()
api_key = os.getenv("ONSHAPE_SECRET_KEY")
print(api_key)