from scripts.deploy_price_feed_consumer import deploy_price_feed_consumer
from scripts.deploy_mocks import INITIAL_ANSWER

def test_price_consumer():
    price_consumer = deploy_price_feed_consumer()
    assert (price_consumer.getLatestPrice() == INITIAL_ANSWER)