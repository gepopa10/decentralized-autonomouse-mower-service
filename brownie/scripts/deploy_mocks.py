from brownie import MockV3Aggregator, accounts

DECIMALS = 8
INITIAL_ANSWER = 500000000000

def deploy_mocks():
    account = accounts[0]
    mock_price_feed = MockV3Aggregator.deploy(DECIMALS, INITIAL_ANSWER, {"from": account})
    return mock_price_feed

