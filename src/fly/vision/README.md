# RFDETR Setup

Simple script to install dependencies into Anaconda's default 'ai' environment.

## Usage

```bash
bash setup_env.sh
```

This will:
- Use the existing `ai` conda environment
- Install: torch, torchvision, numpy, supervision, Pillow, rfdetr (likely only rfdetr needs installing)

## Activate

```bash
conda activate ai
```

## Deactivate

```bash
conda deactivate
```
