from brownie import PriceConsumerV3, accounts, network
from scripts.deploy_mocks import deploy_mocks

def deploy_price_feed_consumer():
    account = accounts[0]
    # if on local network, deploy a mock
    # otherwise, use a regular address
    mock_price_feed_address = ""
    if network.show_active() == "development":
        mock_price_feed = deploy_mocks()
        mock_price_feed_address = mock_price_feed.address
    price_consumer = PriceConsumerV3.deploy(mock_price_feed_address, {"from": account})
    print(price_consumer.getLatestPrice())
    return price_consumer

def main():
    deploy_price_feed_consumer()