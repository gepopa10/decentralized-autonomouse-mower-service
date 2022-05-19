from brownie import decentralized_lawn_mower, APIConsumer, Oracle, Operator, RobotUrlAPIConsumer, robot_camera_ipfs_nft
from scripts.get_account import get_account
import time

node_address = "0x8D1A08a87E03D8342debce270141835f4B946c8b"

def start_mission():
    account = get_account()
    # to work with the last contract deployed
    decentralized_lawn_mower_contract = decentralized_lawn_mower[-1]
    print(decentralized_lawn_mower_contract.mission_fee())
    fee = 0.0099*1e18  # eth
    tx = decentralized_lawn_mower_contract.start_mission(
        {"from": account, "value": fee})
    tx.wait(1)
    print(tx.events["mission_started"]["time"])


def request_robot_url():
    account = get_account()
    api_consumer_contract = APIConsumer[-1]
    tx = api_consumer_contract.requestRobotUrl(
        {"from": account, "gas_limit": 2074044, 'allow_revert': True})
    tx.wait(1)
    print(api_consumer_contract.robot_url())

def request_robot_url_bytes():
    operator_contract = Operator[-1]
    # print(operator_contract)
    account = get_account()
    # print(account)
    api_consumer_contract = RobotUrlAPIConsumer[-1]
    print(api_consumer_contract)
    tx = api_consumer_contract.requestRobotUrlBytes(
        {"from": account, "gas_limit": 2074044, 'allow_revert': True})
    tx.wait(1)
    time.sleep(10)
    print(tx.events)
    # print(tx.events["RequestFulfilled"])
    print(api_consumer_contract.robot_url())
    print(api_consumer_contract.data())

def request_robot_image_uri_bytes():
    operator_contract = Operator[-1]
    # print(operator_contract)
    account = get_account()
    # print(account)
    api_consumer_contract = RobotUrlAPIConsumer[-1]
    print(api_consumer_contract)
    tx = api_consumer_contract.requestRobotImageUriBytes(
        {"from": account, "gas_limit": 2074044, 'allow_revert': True})
    tx.wait(1)
    time.sleep(10)
    print(tx.events)
    # print(tx.events["RequestFulfilled"])
    print("my_uri")
    print(api_consumer_contract.robot_image_uri())

def set_node_fullfilment_permission():
    account = get_account()
    
    oracle_contract = Oracle[-1]
    tx = oracle_contract.setFulfillmentPermission(
        node_address, True, {"from": account})
    tx.wait(1)


def set_operator_fullfilment_permission():
    account = get_account()

    node_address_list = [node_address]
    operator_contract = Operator[-1]
    tx = operator_contract.setAuthorizedSenders(node_address_list, {
                                                  "from": account, "gas_limit": 100000, "allow_revert": True})
    tx.wait(1)

def mint_token():
    account = get_account()

    robot_camera_ipfs_nft_contract = robot_camera_ipfs_nft[-1]
    print(robot_camera_ipfs_nft_contract)
    uri = "ipfs://QmYGKQEWDMXfbBgXe2Qy28zhLxnMnkGwUasWzYfbS4PUqt"

    tx = robot_camera_ipfs_nft_contract.mintToken(uri,
        {"from": account, "gas_limit": 2074044, 'allow_revert': True})

    tx.wait(1)

    print(tx.return_value)

def main():
    # start_mission()
    # set_node_fullfilment_permission()
    # set_operator_fullfilment_permission()
    # request_robot_url()
    # request_robot_url_bytes()
    # mint_token()
    request_robot_image_uri_bytes()
