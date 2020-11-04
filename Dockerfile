FROM python:2.7.18-slim-buster AS base

ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV VIRTUAL_ENV=/opt/venv

FROM base AS builder
RUN apt-get update && \
    apt-get install -y --no-install-recommends gcc build-essential libglib2.0-dev virtualenv

RUN virtualenv --python=python2 $VIRTUAL_ENV
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

COPY requirements.txt ./requirements.txt
RUN pip install -U pip && pip install --no-cache-dir --no-warn-script-location -r ./requirements.txt

FROM base
RUN apt-get update && \
    apt-get install -y libglib2.0-0

ENV PIP_DISABLE_PIP_VERSION_CHECK=1
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
COPY --from=builder /opt/venv /opt/venv

WORKDIR /app
COPY *.py ./

ENTRYPOINT [ "python", "./dfu.py"]