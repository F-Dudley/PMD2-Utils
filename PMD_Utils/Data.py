import os
import json
import numpy as np

from .Devices.Core import FragmentInfo, fragment_type


def load_fragment_metadata(
    fragment_dir: str, metadata_name="fragment_metadata"
) -> FragmentInfo:
    """
    Loads the JSON metadata for fragments in the specified directory.
    """

    assert os.path.exists(
        fragment_dir
    ), f"Fragment directory {fragment_dir} does not exist."

    with open(os.path.join(fragment_dir, f"{metadata_name}.json"), "r") as json_file:
        meta_data = json.load(json_file)

    return meta_data


def load_fragments(
    fragment_dir: str,
    fragment_metadata: FragmentInfo,
    fragment_prefix: str = "frag",
    fragment_suffix: str = ".npy",
) -> tuple[np.ndarray, list[str]]:
    """
    Loads all fragments from the specified directory, and returns them as a single ndarray.
    Also returns the feature names from the metadata.
    """

    fragments = []

    if not os.path.exists(fragment_dir):
        return fragments

    for file_name in os.listdir(fragment_dir):
        if not file_name.startswith(fragment_prefix) or not file_name.endswith(
            fragment_suffix
        ):
            continue

        try:
            fragment = np.load(os.path.join(fragment_dir, file_name))

            if fragment.shape[1] != len(fragment_metadata["feature_names"]):
                raise ValueError(
                    f"Fragment {file_name} has an incompatible shape: "
                    f"{fragment.shape[1]} features found, "
                    f"expected {len(fragment_metadata['feature_names'])}."
                )

            fragments.append(fragment)
        except Exception as e:
            print(f"Error loading fragment {file_name}: {e}")
            continue

    return np.concatenate(fragments, axis=0), fragment_metadata["feature_names"]


def load_fragments_(
    fragment_dir: str, verbose: bool = False
) -> tuple[list[np.ndarray], list[str]]:
    """
    Loads all fragments from the specified directory, and returns them as a single ndarray.
    Also returns the feature names from the metadata.

    @deprecated Use `load_fragments` instead. Only applicable to legacy fragments before metadata was introduced.
    """

    print(
        "Warning: `load_fragments_` is deprecated. Use `load_fragments` with metadata instead."
    )

    fragments = []
    feature_names = []

    if not os.path.exists(fragment_dir):
        return fragments, feature_names

    for file_name in os.listdir(fragment_dir):
        if not file_name.startswith("frag") or not file_name.endswith(".npy"):
            print(
                f"Skipping file {file_name} as it does not match the expected pattern."
            )
            continue

        try:
            fragment = np.load(os.path.join(fragment_dir, file_name))
            if verbose:
                print(f"Loaded fragment {file_name} with shape {fragment.shape}")

            # Extract feature names from the first fragment if not already set
            if not feature_names and fragment.size > 0:
                feature_names = [str(name) for name, _ in fragment[0]]

            if fragment.shape[1] != len(feature_names):
                raise ValueError(
                    f"Fragment {file_name} has an incompatible shape: "
                    f"{fragment.shape[1]} features found, "
                    f"expected {len(feature_names)}."
                )

            fragment = fragment[:, :, 1]
            if verbose:
                print(f"Fragment {file_name} has {fragment.shape} after slicing.")

            fragments.append(fragment)

        except Exception as e:
            print(f"Error loading fragment {file_name}: {e}")
            continue

    if verbose:
        print(f"Loaded {len(fragments)} fragments with feature names: {feature_names}")

    return np.concatenate(fragments, axis=0), feature_names