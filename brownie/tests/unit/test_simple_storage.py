from brownie import accounts
from scripts.deploy import deploy_simple_storage

def test_can_set_number():
    # Arrange
    simple_storage_contract = deploy_simple_storage()
    account = accounts[0]
    expected_number = 777

    # Act
    tx = simple_storage_contract.setNumber(expected_number)
    tx.wait(1)


    # Assert 
    assert simple_storage_contract.getNumber() == expected_number
    assert simple_storage_contract.number() == expected_number
    print(simple_storage_contract.address)

def test_dummy():
    assert True