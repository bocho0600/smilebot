import csv
import cv2
import time

def calculate_and_display_fps(frame, start_time):
    """Calculate and display FPS on the given frame."""
    elapsed_time = time.time() - start_time
    fps = 1.0 / elapsed_time if elapsed_time > 0 else 0
    cv2.putText(frame, f'FPS: {int(fps)}', (20, 30), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (255, 255, 100), 2)
    return fps

class CSVReader:
    def __init__(self, file_name):
        self.file_name = file_name
        self.item_info = []
        self.robot_instruction = [['Aisle', 'Bay', 'Side', 'Height', 'Name']]

    def read_csv(self):
        with open(self.file_name, mode="r", encoding='utf-8-sig') as csv_file:
            self.item_info = list(csv.reader(csv_file))
            # for row in self.item_info:
            #     print(row)  # This line can be removed if you don't need to print each row
        return self.item_info
    
    def RobotInstruction(self):
        robot_instruction2 = []
        for row in self.item_info[1:]:  # Skip the header row
            if row[1] in ["0", "1"]:
                aisle = '1'
                side = 'Left' if row[1] == "0" else 'Right'
            elif row[1] in ["2", "3"]:
                aisle = '2'
                side = 'Left' if row[1] == "2" else 'Right'
            elif row[1] in ["4", "5"]:
                aisle = '3'
                side = 'Left' if row[1] == "4" else 'Right'
            else:
                continue  # Skip rows that do not match any of the conditions

            # Append the transformed row to robot_instruction
            robot_instruction2.append([aisle, row[2], side, row[3], row[4]])
        

        print("Robot Instructions Before:")
        for instruction in self.robot_instruction:
            print(instruction)
        for instruction in robot_instruction2:
            print(instruction)

        # Now we do this to order the priority:
        # Sort by item difficulty
        # Move aisle 1 to the bottom
        # Select the first from each aisle
        # Order be descending row number
        # def item_difficulty(row):
        #     item = row[4]
        #     if item == "Cube" or item == "Mug":
        #         return 1 # easy
        #     elif item == "Weetbots"or item == "Bottle":
        #         return 3 # hard
        #     else:
        #         return 2

        # robot_instruction2 = sorted(robot_instruction2, key = item_difficulty)
        # robot_instruction2 = [r for r in robot_instruction2 if r[0] != '1'] + [r for r in robot_instruction2 if r[0] == '1']
        # robot_instruction2 = [next(r for r in robot_instruction2 if r[3] == '0'), next(r for r in robot_instruction2 if r[3] == '1'), next(r for r in robot_instruction2 if r[3] == '2')]
        # robot_instruction2 = sorted(robot_instruction2, key = lambda r:r[0], reverse=True)

        self.robot_instruction = self.robot_instruction + robot_instruction2

        print("Robot Instructions After:")
        for instruction in self.robot_instruction:
            print(instruction)
        
        return self.robot_instruction


