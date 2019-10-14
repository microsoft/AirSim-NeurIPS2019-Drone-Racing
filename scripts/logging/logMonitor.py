import glob
import os
import time

disqualified_racers = set()
finished_racers = set()
gates_passed = 0
last_gate_passed = "None Passed"

def openfile():
  list_of_files = glob.glob('*.log')
  latest_file = max(list_of_files, key=os.path.getctime)
  print("Opened file: " + latest_file)
  return open(latest_file, "r+")

def follow(thefile):
    thefile.seek(0,2)
    while True:
        line = thefile.readline()
        if not line:
            time.sleep(0.1)
            continue
        yield line  

def main():
  f = openfile()
  for line in follow(f):
    process(line)

def process(line):
  tokens = line.split()
  if len(tokens) != 3 and len(tokens) != 5:
    print("ERROR Bad line: " + line)
    print("Tokens: " + str(tokens))
    return

  if tokens[3] == "disqualified" and tokens[4] == '1' and tokens[2] not in disqualified_racers:
    disqualified_racers.add(tokens[2])
    handleNewDisqualify(tokens[2])
    return

  if tokens[3] == "finished" and tokens[4] == '1' and tokens[2] not in finished_racers:
    finished_racers.add(tokens[2])
    handleNewFinish(tokens[2])
    return

  if tokens[3] == "gates_passed":
    handleGatePassed(tokens[4])

def handleNewDisqualify(racer_name):
  print(racer_name + " has been disqualified!")
  #Start a new race.

def handleNewFinish(racer_name):
  print(racer_name + " has finished!")
  #Start a new race.

def handleGatePassed(number_passed):
  gates_passed = number_passed
  last_gate_passed = "Gate" + format(number_passed - 1, '02d')


if __name__ == "__main__":
  main() 