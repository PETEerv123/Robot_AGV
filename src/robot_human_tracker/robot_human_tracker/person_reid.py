#!/usr/bin/env python3

import cv2
import numpy as np

# torch library
import torch
import torchreid
import torch.nn.functional as F


class PersonReID:
    def __init__(self, device="cuda"):
        self.device = device if torch.cuda.is_available() else "cpu"

        self.model = torchreid.models.build_model(
            name="osnet_x0_25",
            num_classes=1000,
            pretrained=True,
        )

        self.model.eval()
        self.model.to(self.device)

        self.mean = torch.tensor(
            [0.485, 0.456, 0.406],
            dtype=torch.float32,
            device=self.device,
        ).view(1, 3, 1, 1)

        self.std = torch.tensor(
            [0.229, 0.224, 0.225],
            dtype=torch.float32,
            device=self.device,
        ).view(1, 3, 1, 1)

    def extract(self, image, bbox):
        x1, y1, x2, y2 = bbox

        h, w = image.shape[:2]

        x1 = max(0, min(int(x1), w - 1))
        x2 = max(0, min(int(x2), w - 1))
        y1 = max(0, min(int(y1), h - 1))
        y2 = max(0, min(int(y2), h - 1))

        if x2 <= x1 or y2 <= y1:
            return None

        crop = image[y1:y2, x1:x2]

        if crop.size == 0:
            return None

        # BGR -> RGB
        crop = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)

        # OSNet input
        crop = cv2.resize(crop, (128, 256))

        crop = crop.astype(np.float32) / 255.0

        tensor = torch.from_numpy(crop)
        tensor = tensor.permute(2, 0, 1)
        tensor = tensor.unsqueeze(0)

        tensor = tensor.to(self.device)

        tensor = (tensor - self.mean) / self.std

        with torch.no_grad():
            embedding = self.model(tensor)

        # Normalize embedding
        embedding = F.normalize(
            embedding,
            p=2,
            dim=1,
        )

        return embedding

    def similarity(self, target_embedding, image, bbox):
        if target_embedding is None:
            return 0.0

        embedding = self.extract(image, bbox)

        if embedding is None:
            return 0.0

        score = torch.sum(
            target_embedding * embedding,
            dim=1,
        )

        return float(score.item())
