from brownie import decentralized_lawn_mower, accounts, config, Operator, RobotUrlAPIConsumer, robot_camera_ipfs_nft
# import os
from scripts.get_account import get_account

link_address = "0xa36085F69e2889c224210F603D836748e7dC0088"

def deploy_operator():
    account = get_account()

    operator_contract = Operator.deploy(link_address, account, {"from": account})

    return operator_contract

def deploy_robot_url_api_consumer():
    account = get_account()

    operator_address = Operator[-1]
    robot_url_api_consumer_contract = RobotUrlAPIConsumer.deploy(link_address, operator_address, {"from": account})

    return robot_url_api_consumer_contract

def deploy_decentralized_lawn_mower():
    # account = accounts[0]
    # account = accounts.add(os.getenv("PRIVATE_KEY"))
    # account = accounts.add(config["wallets"]["from_key"])
    account = get_account()

    print(account)
    decentralized_lawn_mower_contract = decentralized_lawn_mower.deploy({"from": account})
    print(decentralized_lawn_mower_contract.mission_fee())
    print(decentralized_lawn_mower_contract.address)
    return decentralized_lawn_mower_contract

def robot_camera_ipfs_nft_contract():
    account = get_account()

    robot_camera_ipfs_nft_contract = robot_camera_ipfs_nft.deploy({"from": account})

    return robot_camera_ipfs_nft_contract

def main():
    # deploy_decentralized_lawn_mower()
    # deploy_operator()
    deploy_robot_url_api_consumer()
    # robot_camera_ipfs_nft_contract()