package main

import (
	"fmt"
	"math"

	rvo "github.com/bolom009/rvo2-go/src/rvosimulator"
	rl "github.com/gen2brain/raylib-go/raylib"
)

var (
	goals []*rvo.Vector2
)

func init() {
	goals = make([]*rvo.Vector2, 0)
}

func setupScenario(sim *rvo.RVOSimulator) {
	sim.SetTimeStep(0.25)
	sim.SetAgentDefaults(15.0, 10, 10.0, 10.0, 1.5, 2.0, &rvo.Vector2{}) // where is velocity property ?

	agentNum := 20
	goals = make([]*rvo.Vector2, agentNum)
	for i := 0; i < agentNum; i++ {
		position := &rvo.Vector2{
			X: math.Cos(float64(i) * 2.0 * math.Pi / float64(agentNum)),
			Y: math.Sin(float64(i) * 2.0 * math.Pi / float64(agentNum)),
		}
		id, err := sim.AddDefaultAgent(position)

		if !err {
			sim.SetAgentGoal(id, position)
		}
	}
}

func updateVisualization(sim *rvo.RVOSimulator, camera rl.Camera2D) {

}

func setPreferredVelocities(sim *rvo.RVOSimulator) {
	for i := 0; i < sim.GetNumAgents(); i++ {
		// goal - agentPosition
		goalVector := sim.GetAgentGoalVector(i)

		if rvo.Sqr(goalVector) > 1 {
			goalVector = rvo.Normalize(goalVector)
		}

		sim.SetAgentPrefVelocity(i, goalVector)
	}
}

func main() {
	sim := rvo.NewEmptyRVOSimulator()
	setupScenario(sim)

	for {
		if sim.IsReachedGoal() {
			fmt.Printf("Goal \n ")
			break
		}

		updateVisualization(sim)

		setPreferredVelocities(sim)
		sim.DoStep()
	}
}
