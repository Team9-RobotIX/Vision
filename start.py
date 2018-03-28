import planning
robots = [
    [(150,45,45),(255,110,110),(0,0,225),(50,200,295),2.5,2.5]
]
controller = planning.Controller(robots)
controller.run()
