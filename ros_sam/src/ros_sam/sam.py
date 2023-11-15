from pathlib import Path
import torch

from segment_anything import sam_model_registry, \
                             SamPredictor

# Free up GPU memory before loading the model
import gc


class SAM():
    def __init__(self, model_type, cuda_device=None) -> None:
        model_dir   = Path(Path(__file__).parent.parent.parent / 'models').absolute()
        checkpoints = list(model_dir.glob(f'sam_{model_type}_*.pth'))

        if len(checkpoints) == 0:
            raise RuntimeError(f'No matching checkpoints for SAM model "{model_type}" was found in "{model_dir}"')
        
        if len(checkpoints) > 1:
            raise RuntimeError(f'No unique checkpoint for SAM model "{model_type}" found in "{model_dir}"')
        
        gc.collect()
        torch.cuda.empty_cache()

        sam_model    = sam_model_registry[model_type](checkpoint=checkpoints[0])
        self._device = cuda_device

        if self._device is not None:
            sam_model.to(device=self._device)

        self._predictor = SamPredictor(sam_model)
    
    def __del__(self):
        gc.collect()
        torch.cuda.empty_cache()

    def segment(self, img, points, point_labels, boxes=None, multimask=True):
        self._predictor.set_image(img)        
        return self._predictor.predict(
            point_coords=points,
            point_labels=point_labels,
            box=boxes,
            multimask_output=multimask,
        )
