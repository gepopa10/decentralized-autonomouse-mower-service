from brownie import accounts, network
from scripts.deploy_decentralized_lawn_mower import deploy_decentralized_lawn_mower
import pytest
from scripts.get_account import get_account

def test_GIVEN_fee_is_payed_WHEN_start_mission_THEN_mission_started_event_called():
    # Arrange
    decentralized_lawn_mower_contract = deploy_decentralized_lawn_mower()
    account = get_account()
    fee = 0.0099*1e18

    print(account.balance()/1e18)

    # Act
    tx = decentralized_lawn_mower_contract.start_mission(
        {"from": account, "value": fee})
    tx.wait(1)

    # Assert
    assert 'mission_started' in tx.events
    print(decentralized_lawn_mower_contract.address)


def test_GIVEN_fee_is_not_payed_WHEN_start_mission_THEN_mission_started_event_not_called():
    # Arrange
    decentralized_lawn_mower_contract = deploy_decentralized_lawn_mower()
    account = get_account()
    fee = 0.0098*1e18
    # Act
    with pytest.raises(ValueError):
        tx = decentralized_lawn_mower_contract.start_mission(
            {"from": account, "value": fee})
        tx.wait(1)

        # Assert
        assert not ('mission_started' in tx.events)

    print(decentralized_lawn_mower_contract.address)
