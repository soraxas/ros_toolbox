from collections import OrderedDict
from typing import Callable, Optional, List, Dict
from typing import Iterator, Type, Tuple

import rosgraph
import rospy
from genpy.message import Message
from rospy import Publisher


def _get_subscribers(topic_path: str):
    """Get all subscribers under the given topic."""
    ros_master = rosgraph.Master("/rostopic")
    topic_path = rosgraph.names.script_resolve_name("rostopic", topic_path)
    state = ros_master.getSystemState()
    subs = []
    for sub in filter(lambda x: x[0] == topic_path, state[1]):
        subs.extend(sub[1])
    return subs


def construct_publisher(
        topic_path: str, msg_type: Type[Message], timeout: float = 1, **kwargs
):
    """
    Construct a publisher instance with the given topic,
    and wait at most x second before returning, to ensure
    that the publisher is ready.
    """
    if "queue_size" not in kwargs:
        # silence warning
        kwargs["queue_size"] = 10

    pub = rospy.Publisher(topic_path, msg_type, **kwargs)
    num_subs = len(_get_subscribers(topic_path))
    for i in range(int(timeout / 0.1) + 1):
        num_cons = pub.get_num_connections()
        if num_cons == num_subs:
            return pub
        rospy.sleep(0.1)
    print(RuntimeError("Timeout. failed to get publisher...?"))
    return pub


def construct_subscriber(
        topic_path: str,
        msg_type: Type[Message],
        callback: Callable[[Message], None],
        timeout: float = 1,
        **kwargs
):
    """
    Construct a publisher instance with the given topic,
    and wait at most x second before returning, to ensure
    that the subscriber is ready.
    """
    sub = rospy.Subscriber(topic_path, msg_type, callback=callback, **kwargs)
    rospy.sleep(timeout)
    # for i in range(int(timeout/0.1)+1):
    #     sub.getNumSubscribers()
    #     num_cons = sub.get_num_connections()
    #     if num_cons == num_subs:
    #         return sub
    #     rospy.sleep(0.1)
    # print(RuntimeError("Timeout. failed to get publisher...?"))
    return sub


class TrackLatestPublishedMsg:
    """
    Container that will always keep track of the latest message.
    """

    def __init__(self, topic_name: str, msg_type: Type[Message]):
        self.last_msg = None

        def _callback(msg: Message):
            self.last_msg = msg

        self.sub = construct_subscriber(topic_name, msg_type, _callback)

    def get(self):
        return self.last_msg


class PublishersManager:
    """
    A wrapper class that will publish to each individual publisher. Useful to
    publish message in a list of publisher

    e.g.
    ```
    from std_msgs.msg import Float64

    j_publishers = ros_toolbox.PublishersManager(
        ((f"/j2n6s300/joint_{i}_position_controller/command", Float64)
            for i in range(1, 6+1))
        )

    j_publishers.publish_by_order([...])
    ```
    """

    def __init__(
            self,
            topic_names: Iterator[Tuple[str, Type[Message]]],
            ignore_non_exist: bool = False,
            predefined_names: Optional[List[str]] = None,
    ):
        self.publishers = OrderedDict()

        # use topic name as key
        for topic_name, msg_type in topic_names:
            self.publishers[topic_name] = construct_publisher(topic_name, msg_type)

        # use predefined names as key
        self.publishers_by_key = None
        if predefined_names is not None:
            self.publishers_by_key = OrderedDict()
            for (topic_name, msg_type), key_name in zip(
                    topic_names, predefined_names
            ):
                self.publishers_by_key[key_name] = construct_publisher(
                    topic_name, msg_type
                )

        self.ignore_non_exist = ignore_non_exist

    def publish_by_predefined_name(self, names: List[str], msgs: List[Message]):
        """Publish a list of messages based on some predefined names (as keys)."""
        if self.publishers_by_key is None:
            raise ValueError(
                "Predefined names are not given when this object is constructed."
            )
        self.__publish_via_key(
            self.publishers_by_key, names, msgs, self.ignore_non_exist
        )

    def publish_by_topic_name(self, topic_names: List[str], msgs: List[Message]):
        """Publish a list of messages based on topic names."""
        self.__publish_via_key(
            self.publishers, topic_names, msgs, self.ignore_non_exist
        )

    @staticmethod
    def __publish_via_key(
            publishers: Dict[str, Publisher],
            keys: List[str],
            msgs: List[Message],
            ignore_non_exists: bool,
    ):
        """Helper function, where it will publish a list of messages with error
        handling."""
        for key, msg in zip(keys, msgs):
            try:
                publishers[key].publish(msg)
            except KeyError as e:
                if not ignore_non_exists:
                    raise e

    def publish_by_order(self, msgs: List[Message]):
        """Publish a list of messages based on their order."""
        for msg, pub in zip(msgs, self.publishers.values()):
            # assert isinstance(msg, Message)
            pub.publish(msg)
