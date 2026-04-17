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

Then you can run the default installation requirements in the fly directory to use the (ai) venv for all programs.