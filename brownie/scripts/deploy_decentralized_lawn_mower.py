from brownie import decentralized_lawn_mower, accounts, config
# import os
from scripts.get_account import get_account

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

def main():
    deploy_decentralized_lawn_mower()