FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy app code
#COPY . .

COPY app/ ./app
# Start the FastAPI server
#CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
# Expose port (Railway automatically sets $PORT)
EXPOSE 8000

# Run the FastAPI app using Uvicorn
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
