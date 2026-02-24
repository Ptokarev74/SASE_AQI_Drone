# --- Stage 1: Build environment ---
FROM python:3.12-slim AS builder

WORKDIR /app

# Install build dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies into a virtualenv
RUN python -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

COPY bridge/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# --- Stage 2: Runtime environment ---
FROM python:3.12-slim

WORKDIR /app

# Copy the virtualenv from the builder stage
COPY --from=builder /opt/venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"
ENV PYTHONPATH=.

# Create a non-root user for security
RUN useradd -m droneuser && chown -R droneuser:droneuser /app
USER droneuser

# Copy the bridge package and documentation
COPY --chown=droneuser:droneuser bridge/ ./bridge/
COPY --chown=droneuser:droneuser PROTOCOL.md .

# Expose the WebSocket port
EXPOSE 8765

# Launch the bridge using the canonical module entrypoint
ENTRYPOINT ["python", "-m", "bridge"]
