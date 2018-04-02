import planning
robots = [
    [(150,55,55),(255,100,100),(-20,17,235),(20,57,275),3,3],
    [(73,151,55),(113,191,95),(-20,137,235),(20,177,275),3,1]
]
controller = planning.Controller(robots)
controller.run()
