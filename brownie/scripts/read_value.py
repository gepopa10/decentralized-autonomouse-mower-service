from brownie import decentralized_lawn_mower, accounts, config

def read_contract():
    account = accounts.add(config["wallets"]["from_key"])
    decentralized_lawn_mower_contract = decentralized_lawn_mower[-1] # to work with the last contract deployed
    print(decentralized_lawn_mower_contract.mission_fee())
    fee = 0.0099*1e18
    tx = decentralized_lawn_mower_contract.start_mission({"from": account, "value": fee})
    tx.wait(1)

def main():
    read_contract()