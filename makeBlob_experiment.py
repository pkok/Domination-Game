import cPickle as pickle
from collections import defaultdict

class AutoVivification(dict):
	"""Implementation of perl's autovivification feature."""
	def __getitem__(self, item):
		try:
			return dict.__getitem__(self, item)
		except KeyError:
			value = self[item] = type(self)()
			return value


if __name__ == "__main__":
	state_action_pairs = {}
	blobfile = open("my_agent_experiment_blob", 'wb')
	pickle.dump(state_action_pairs, blobfile, pickle.HIGHEST_PROTOCOL)