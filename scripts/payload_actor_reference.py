"""Fixed successful-state rehearsal through RSL-RL's auxiliary-loss interface."""
import hashlib
import json
import math
from pathlib import Path

import torch


class ActorReferenceLoss:
    # PPO's symmetry slot provides a pre-backward auxiliary loss hook. No
    # symmetry augmentation is performed; this is supervised actor rehearsal.
    use_mirror_loss = True

    def __init__(self, actor, corpus, weight):
        if not math.isfinite(weight) or weight <= 0:
            raise ValueError('reference weight must be positive and finite')
        if actor.is_recurrent or actor.obs_normalization:
            raise ValueError('reference rehearsal requires an unnormalized feed-forward actor')
        data = json.loads(Path(corpus).read_text())
        device = next(actor.parameters()).device
        self.observations = torch.tensor(data['observations'], dtype=torch.float32, device=device)
        self.actions = torch.tensor(data['actions'], dtype=torch.float32, device=device)
        n = len(self.observations)
        if (n < 2 or self.observations.shape != (n, 480) or self.actions.shape != (n, 29)
                or not torch.isfinite(self.observations).all() or not torch.isfinite(self.actions).all()):
            raise ValueError('invalid reference corpus')
        if not data.get('all_sources_passed') or not data.get('moving_samples') or not data.get('stopped_samples'):
            raise ValueError('reference corpus must contain qualified movement and stopping')
        self.mirror_loss_coeff = weight
        self.cursor = 0
        self.updates = 0
        self.total_loss = 0.
        self.sha256 = hashlib.sha256(Path(corpus).read_bytes()).hexdigest()

    def augment_batch(self, batch, original_batch_size):
        pass

    def compute_loss(self, actor, batch, original_batch_size):
        # Deterministic cycling avoids changing PPO's random-number stream.
        count = min(256, len(self.observations))
        ids = (torch.arange(count, device=self.observations.device) + self.cursor) % len(self.observations)
        self.cursor = (self.cursor + count) % len(self.observations)
        loss = (actor.mlp(self.observations[ids]) - self.actions[ids]).square().mean()
        self.updates += 1
        self.total_loss += float(loss.detach())
        return loss

    def audit(self):
        return dict(corpus_sha256=self.sha256, weight=self.mirror_loss_coeff,
                    updates=self.updates, mean_mse=self.total_loss/max(1,self.updates),
                    samples=len(self.observations), scope='Fixed native successful-state actor rehearsal; PPO auxiliary slot, no symmetry augmentation')
