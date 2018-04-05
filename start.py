import planning
robots = [
    #[(145,50,50),(230,100,100),(-20,17,235),(20,57,275),3,3],
    [(63,141,45),(113,191,95),(-20,127,225),(20,177,275),3,1]
]
controller = planning.Controller(robots)
controller.run()
