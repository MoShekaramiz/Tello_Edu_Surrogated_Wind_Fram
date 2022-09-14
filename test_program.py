import movement as mv

drone = mv.movement()

drone.move(fwd=200)
drone.go_to(0, 0)
drone.land()
