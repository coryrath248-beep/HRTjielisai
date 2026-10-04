import torch
import torch.nn.functional as F


def expand_appended_observations(model, state, appended):
    """Preserve a policy when new observations are appended to its input."""
    if not appended:
        return state
    state = state.copy()
    expected = model.state_dict()
    for key in ("actor.0.weight", "critic.0.weight"):
        loaded = state[key]
        target = expected[key]
        if loaded.shape == target.shape:
            continue
        if loaded.shape[0] != target.shape[0] or loaded.shape[1] + appended != target.shape[1]:
            raise ValueError(f"Cannot append {appended} observations to {key}: {loaded.shape} -> {target.shape}")
        expanded = torch.zeros_like(target)
        expanded[:, :loaded.shape[1]] = loaded
        state[key] = expanded
    return state


class ActorCritic(torch.nn.Module):

    def __init__(self, num_act, num_obs, num_privileged_obs):
        super().__init__()
        self.critic = torch.nn.Sequential(
            torch.nn.Linear(num_obs + num_privileged_obs, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 1),
        )
        self.actor = torch.nn.Sequential(
            torch.nn.Linear(num_obs, 256),
            torch.nn.ELU(),
            torch.nn.Linear(256, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, 128),
            torch.nn.ELU(),
            torch.nn.Linear(128, num_act),
        )
        self.logstd = torch.nn.parameter.Parameter(torch.full((1, num_act), fill_value=-2.0), requires_grad=True)

    def act(self, obs):
        action_mean = self.actor(obs)
        action_std = torch.exp(self.logstd).expand_as(action_mean)
        return torch.distributions.Normal(action_mean, action_std)

    def est_value(self, obs, privileged_obs):
        critic_input = torch.cat((obs, privileged_obs), dim=-1)
        return self.critic(critic_input).squeeze(-1)
