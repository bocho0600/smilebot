# state_machine.py
import sys
from enum import Enum

MAIN_STATE = Enum('MAIN_STATE', [
      'INIT', 
      'FIND_ORDER',
      'TAKE_ORDER',
      'TRANSIT_ORDER',
])

SUB_STATE = Enum('SUB_STATE', [
      # States for INIT
      'CALIBRATE_LOCATION',
      'LOAD_ORDER',


      # States for FIND_ORDER
      'FIND_ENTRY',
      'MOVE_TO_ENTRY',
      'FIND_AISLE',
      'FIND_BAY',
      'MOVE_TO_ROW',
      'SWAP_ENTRY',

      # States for TAKE_ORDER
      'FIND_ITEM',
      'PICK_ITEM',

      # States for TRANSIT_ORDER
      'FIND_EXIT',
      'EXIT_ROW',
      'FIND_PACIKINGBAY',
      'MOVE_TO_PACKINGBAY',
      'DROP_ITEM',
      'EXIT_PACKINGBAY',

      # States for all
      'LOST',
]) 


def print_state(main_state, task):
      sys.stdout.write(f"\r")
      sys.stdout.flush()
      sys.stdout.write(f"\rSTATE: {main_state.name}, TASK: {task.name}")
      sys.stdout.flush()
