# Multi-stage build for optimized FPM server image

# Stage 1: Builder
FROM python:3.11-slim as builder

WORKDIR /build

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir --user -r requirements.txt

# Stage 2: Runtime
FROM python:3.11-slim

LABEL maintainer="FPM Team"
LABEL description="Fourier Ptychography Microscope Processing Server"
LABEL version="1.0.0"

WORKDIR /app

# Install runtime dependencies (OpenCV requirements)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libgl1 \
    && rm -rf /var/lib/apt/lists/*

# Copy Python packages from builder
COPY --from=builder /root/.local /root/.local

# Copy application code
COPY config.py .
COPY server.py .
COPY reconstruction.py .
COPY utils.py .
COPY visualize.py .
COPY imgload.py .

# Create data directory
RUN mkdir -p /app/data

# Add Python packages to PATH
ENV PATH=/root/.local/bin:$PATH
ENV PYTHONUNBUFFERED=1

# Health check endpoint
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD python -c "import requests; requests.get('http://localhost:8000/health', timeout=5).raise_for_status()"

# Expose server port
EXPOSE 8000

# Default command: Run FastAPI server
CMD ["python", "server.py"]