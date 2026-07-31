#!/usr/bin/env python3
"""Headless text query for FindAnything.

Same CLIP text embedding that language_feature_node's Qt GUI publishes, but from
the command line -- useful over ssh or when scripting a bag replay:

    python tools/language_query.py "a wooden chair" --mode Activations
"""

import argparse
import inspect
import os
import sys

import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile

import language_feature_node
from language_feature_node.f3rm_features import clip
from language_feature_node.f3rm_features.clip.clip import tokenize
from language_feature_node.f3rm_features.clip.simple_tokenizer import SimpleTokenizer
from language_feature_node.f3rm_features.clip_extract import CLIPArgs
from language_feature_msgs.msg import Tensor


def bpe_path() -> str:
    share = os.path.join(get_package_share_directory("language_feature_node"),
                         "bpe_simple_vocab_16e6.txt.gz")
    if os.path.exists(share):
        return share
    module_dir = os.path.dirname(inspect.getfile(language_feature_node))
    return os.path.join(module_dir, "f3rm_features", "clip", "bpe_simple_vocab_16e6.txt.gz")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("query", help="text query, e.g. 'a wooden chair'")
    parser.add_argument("--mode", default="Activations", choices=["Activations", "Colors"],
                        help="Activations: colour meshes by similarity to the query. "
                             "Colors: back to RGB colouring.")
    parser.add_argument("--timeout", type=float, default=10.0,
                        help="seconds to wait for the okvis node to subscribe")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("language_query")
    pub = node.create_publisher(Tensor, "/language_processor/embedding", QoSProfile(depth=1))

    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
    node.get_logger().info(f"Encoding on {device}")
    model, _ = clip.load(CLIPArgs.model_name, device=device, jit=False)
    model.half()

    token = tokenize(args.query, SimpleTokenizer(bpe_path=bpe_path())).to(device)
    with torch.no_grad():
        embedding = model.encode_text(token)
        embedding /= embedding.norm(dim=-1, keepdim=True)

    msg = Tensor()
    msg.header.frame_id = "string_embedding"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.query = args.query
    msg.display_mode = args.mode
    msg.tensor = embedding.to(torch.float32).cpu().reshape(-1).tolist()

    waited = 0.0
    while pub.get_subscription_count() == 0 and waited < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        waited += 0.1
    if pub.get_subscription_count() == 0:
        node.get_logger().error("no subscriber on /language_processor/embedding -- is okvis running?")
        return 1

    pub.publish(msg)
    # let the middleware flush before we tear the context down
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.get_logger().info(f"published {len(msg.tensor)}-D embedding for '{args.query}' ({args.mode})")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
