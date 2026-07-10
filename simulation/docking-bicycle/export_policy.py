"""Export the trained SB3 SAC actor to TorchScript so it can be loaded inside the Isaac Lab runtime
(which has a different numpy than the training env -> SB3's pickle load fails with numpy._core).
TorchScript is self-contained and numpy-agnostic. Produces runs/policy_ts.pt: obs(5) -> action(2).

Usage:
    python export_policy.py                          # export the default (latest) checkpoint
    python export_policy.py runs/sac_bicycle_v4.zip  # export a specific checkpoint
    python export_policy.py runs/sac_bicycle_v4.zip runs/policy_ts.pt   # + explicit output path

NB: pass the explicit .zip path -- SB3's SAC.load appends '.zip' to a bare name, and a leftover
extracted 'runs/sac_bicycle*/' directory then shadows the file (IsADirectoryError)."""
import sys

import torch
from stable_baselines3 import SAC

MODEL = sys.argv[1] if len(sys.argv) > 1 else "runs/sac_bicycle_v4.zip"
OUT = sys.argv[2] if len(sys.argv) > 2 else "runs/policy_ts.pt"

model = SAC.load(MODEL, device="cpu")
policy = model.policy.eval()


class DetActor(torch.nn.Module):
    def __init__(self, policy):
        super().__init__()
        self.policy = policy

    def forward(self, obs):
        return self.policy._predict(obs, deterministic=True)


wrap = DetActor(policy)
example = torch.zeros(1, 5, dtype=torch.float32)
with torch.no_grad():
    traced = torch.jit.trace(wrap, example)
    a_ref = wrap(example)
    a_ts = traced(example)
    assert torch.allclose(a_ref, a_ts, atol=1e-5), "trace mismatch"
traced.save(OUT)
print(f"exported {OUT} from {MODEL} | example action:", a_ts.numpy().round(3))
