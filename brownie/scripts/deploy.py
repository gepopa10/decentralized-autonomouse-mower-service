from brownie import SimpleStorage, accounts

def deploy_simple_storage():
    account = accounts[0]
    simple_storage_contract = SimpleStorage.deploy({"from": account})
    print(simple_storage_contract.number())
    print(simple_storage_contract.address)
    return simple_storage_contract

    # simple_storage_contract.setNumber(777, {"from": account})
    # print(simple_storage_contract.number())

def main():
    deploy_simple_storage()