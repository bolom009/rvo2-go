package rvo2_go

import "errors"

var (
	Sim *RVOSimulator
)

// RVOSimulator defines the simulation. The main struct of the library that contains all simulation functionality.
type RVOSimulator struct {
	TimeStep         float32
	Agents           []*Agent
	Obstacles        [][]*Vector2
	ObstacleVertices []*Obstacle
	KdTree           *KdTree
	DefaultAgent     *Agent
	GlobalTime       float32
	NextAgentID      uint16
}

// NewRVOSimulator constructs a simulator instance and sets the default properties for any new agent that is added
func NewRVOSimulator(timeStep, neighborDist float32, maxNeighbors uint16, timeHorizon, timeHorizonObst, radius, maxSpeed float32, velocity *Vector2) *RVOSimulator {
	kdTree := NewKdTree()
	defaultAgent := NewEmptyAgent()

	defaultAgent.MaxNeighbors = maxNeighbors
	defaultAgent.MaxSpeed = maxSpeed
	defaultAgent.NeighborDist = neighborDist
	defaultAgent.Radius = radius
	defaultAgent.TimeHorizon = timeHorizon
	defaultAgent.TimeHorizonObst = timeHorizonObst
	defaultAgent.Velocity = velocity

	sim := &RVOSimulator{
		TimeStep:         timeStep,
		Agents:           make([]*Agent, 0),
		Obstacles:        make([][]*Vector2, 0),
		ObstacleVertices: make([]*Obstacle, 0),
		KdTree:           kdTree,
		DefaultAgent:     defaultAgent,
		GlobalTime:       0.0,
		NextAgentID:      0,
	}
	Sim = sim

	return sim
}

// NewEmptyRVOSimulator constructs a empty simulator instance
func NewEmptyRVOSimulator() *RVOSimulator {
	kdTree := NewKdTree()
	defaultAgent := NewEmptyAgent()

	sim := &RVOSimulator{
		TimeStep:         0,
		Agents:           make([]*Agent, 0),
		Obstacles:        make([][]*Vector2, 0),
		ObstacleVertices: make([]*Obstacle, 0),
		KdTree:           kdTree,
		DefaultAgent:     defaultAgent,
		GlobalTime:       0.0,
	}
	Sim = sim
	return sim
}

// AddDefaultAgent adds a new agent with default properties to the simulation.
func (rvo *RVOSimulator) AddDefaultAgent(position *Vector2) (uint16, error) {

	if rvo.DefaultAgent == nil {
		return 0, errors.New("empty default rvo agent")
	}

	agent := NewEmptyAgent()
	agent.Position = position
	agent.MaxNeighbors = rvo.DefaultAgent.MaxNeighbors
	agent.MaxSpeed = rvo.DefaultAgent.MaxSpeed
	agent.NeighborDist = rvo.DefaultAgent.NeighborDist
	agent.Radius = rvo.DefaultAgent.Radius
	agent.TimeHorizon = rvo.DefaultAgent.TimeHorizon
	agent.TimeHorizonObst = rvo.DefaultAgent.TimeHorizonObst
	agent.Velocity = rvo.DefaultAgent.Velocity
	agent.ID = rvo.NextAgentID

	rvo.Agents = append(rvo.Agents, agent)
	rvo.NextAgentID++

	return agent.ID, nil
}

// AddAgent adds a new agent to the simulation.
func (rvo *RVOSimulator) AddAgent(position *Vector2, neighborDist float32, maxNeighbors uint16, timeHorizon,
	timeHorizonObst, radius, maxSpeed float32, velocity *Vector2) (uint16, bool) {

	agent := NewEmptyAgent()
	agent.Position = position
	agent.MaxNeighbors = maxNeighbors
	agent.MaxSpeed = maxSpeed
	agent.NeighborDist = neighborDist
	agent.Radius = radius
	agent.TimeHorizon = timeHorizon
	agent.TimeHorizonObst = timeHorizonObst
	agent.Velocity = velocity
	agent.ID = rvo.NextAgentID

	rvo.Agents = append(rvo.Agents, agent)
	rvo.NextAgentID++

	return agent.ID, false
}

// RemoveAgent : Remove agent by agentNo
//func (rvo *RVOSimulator) RemoveAgent(agentNo uint16) bool {
//
//	copy(rvo.Agents[agentNo:], rvo.Agents[agentNo+1:])
//	rvo.Agents[len(rvo.Agents)-1] = nil
//	rvo.Agents = rvo.Agents[:len(rvo.Agents)-1]
//
//	//rvo.Agents = append(rvo.Agents[:agentNo], rvo.Agents[agentNo+1:]...)
//
//	return false
//}

// DisableAgent disable specific agent and exclude from simulation
func (rvo *RVOSimulator) DisableAgent(agentNo uint16) {
	rvo.Agents[agentNo].active = false
}

// GetAgentNoByID get specific agent by id
func (rvo *RVOSimulator) GetAgentNoByID(id uint16) (uint16, bool) {
	for i := uint16(0); i < rvo.GetNumAgents(); i++ {
		if rvo.GetAgent(i).ID == id {
			return i, true
		}
	}
	return 0, false
}

// AddObstacle adds a new obstacle to the simulation.
func (rvo *RVOSimulator) AddObstacle(vertices []*Vector2) (uint16, error) {

	// add obstacle
	rvo.Obstacles = append(rvo.Obstacles, vertices)

	// add obstacle vertices
	if len(vertices) < 2 {
		return 0, errors.New("invalid vertices length")
	}

	// 一つ一つ大きなObstacleはObstacleNoとして管理
	obstacleNo := uint16(len(rvo.ObstacleVertices))

	// Obstacleを一点ずつ置いて行って形を作る
	for i := 0; i < len(vertices); i++ {
		obstacle := NewEmptyObstacle()
		obstacle.Point = vertices[i]

		// NextとPrevObstacleをセット
		if i != 0 {
			obstacle.PrevObstacle = rvo.ObstacleVertices[len(rvo.ObstacleVertices)-1]
			obstacle.PrevObstacle.NextObstacle = obstacle
		}

		if i == len(vertices)-1 {
			obstacle.NextObstacle = rvo.ObstacleVertices[obstacleNo]
			obstacle.NextObstacle.PrevObstacle = obstacle
		}

		var ti int
		if i == len(vertices)-1 {
			ti = 0
		} else {
			ti = i + 1
		}

		obstacle.UnitDir = Normalize(Sub(vertices[ti], vertices[i]))

		var ki int
		if i == 0 {
			ki = len(vertices) - 1
		} else {
			ki = i - 1
		}

		// 凸かどうか　？
		if len(vertices) == 2 {
			obstacle.IsConvex = true
		} else {
			obstacle.IsConvex = LeftOf(vertices[ki], vertices[i], vertices[ti]) >= 0.0
		}

		obstacle.ID = len(rvo.ObstacleVertices)

		rvo.ObstacleVertices = append(rvo.ObstacleVertices, obstacle)

	}

	return obstacleNo, nil
}

// DoStep lets the simulator perform a simulation step and
// updates the two-dimensional position and two-dimensional velocity of each agent.
func (rvo *RVOSimulator) DoStep() {
	rvo.KdTree.BuildAgentTree()

	for _, agent := range rvo.Agents {
		if !agent.active {
			continue
		}

		// agentのneighborsを計算
		agent.ComputeNeighbors()
		// agentの速度を計算
		agent.ComputeNewVelocity(rvo.TimeStep)
		// agentを更新
		agent.Update(rvo.TimeStep)
	}

	// globaltimeを更新
	rvo.GlobalTime += rvo.TimeStep
}

// IsReachedGoal method to check is all agent in simulation reached their goal
func (rvo *RVOSimulator) IsReachedGoal() bool {
	/* Check if all agents have reached their goals. */
	for i := uint16(0); i < rvo.GetNumAgents(); i++ {
		if !rvo.GetAgentActive(i) {
			continue
		}

		if !rvo.IsAgentReachedGoal(i) {
			return false
		}
	}
	return true
}

// IsAgentReachedGoal method to check is specific agent reached the goal
func (rvo *RVOSimulator) IsAgentReachedGoal(agentNo uint16) bool {
	/* Check if agent have reached their goals. */
	if Sqr(Sub(rvo.GetAgentGoal(agentNo), rvo.GetAgentPosition(agentNo))) > rvo.GetAgentRadius(agentNo)*rvo.GetAgentRadius(agentNo) {
		return false
	}
	return true
}

// GetAgentGoalVector returns the specified agent two-dimensional goal
func (rvo *RVOSimulator) GetAgentGoalVector(agentNo uint16) *Vector2 {
	return Normalize(Sub(rvo.GetAgentGoal(agentNo), rvo.GetAgentPosition(agentNo)))
}

// GetAgentActive returns the specified agent active state
func (rvo *RVOSimulator) GetAgentActive(agentNo uint16) bool {
	return rvo.Agents[agentNo].active
}

// GetAgents returns list of all agents in simulation (active and non-active)
func (rvo *RVOSimulator) GetAgents() []*Agent {
	return rvo.Agents
}

// GetAgent returns specific agent by id
func (rvo *RVOSimulator) GetAgent(agentNo uint16) *Agent {
	return rvo.Agents[agentNo]
}

// GetAgentAgentNeighbor returns the specified agent neighbor of the specified agent.
func (rvo *RVOSimulator) GetAgentAgentNeighbor(agentNo, neighborNo uint16) uint16 {
	return rvo.Agents[agentNo].AgentNeighbors[neighborNo].Agent.ID
}

// GetAgentMaxNeighbors returns the maximum neighbor count of a specified agent.
func (rvo *RVOSimulator) GetAgentMaxNeighbors(agentNo uint16) uint16 {
	agent := rvo.Agents[agentNo]
	return agent.MaxNeighbors
}

// GetAgentMaxSpeed returns the maximum speed of a specified agent.
func (rvo *RVOSimulator) GetAgentMaxSpeed(agentNo uint16) float32 {
	agent := rvo.Agents[agentNo]
	return agent.MaxSpeed
}

// GetAgentNeighborDist returns the maximum neighbor distance of a specified agent.
func (rvo *RVOSimulator) GetAgentNeighborDist(agentNo uint16) float32 {
	agent := rvo.Agents[agentNo]
	return agent.NeighborDist
}

// GetAgentNumAgentNeighbors returns the count of agent neighbors taken into account to
// compute the current velocity for the specified agent.
func (rvo *RVOSimulator) GetAgentNumAgentNeighbors(agentNo uint16) int {
	agent := rvo.Agents[agentNo]
	return len(agent.AgentNeighbors)
}

// GetAgentNumObstacleNeighbors returns the count of obstacle neighbors taken into account to
// compute the current velocity for the specified agent.
func (rvo *RVOSimulator) GetAgentNumObstacleNeighbors(agentNo uint16) int {
	agent := rvo.Agents[agentNo]
	return len(agent.ObstacleNeighbors)
}

// GetAgentNumORCALines returns the count of ORCA constraints used to compute the
// current velocity for the specified agent.
func (rvo *RVOSimulator) GetAgentNumORCALines(agentNo uint16) int {
	agent := rvo.Agents[agentNo]
	return len(agent.OrcaLines)
}

// GetAgentObstacleNeighbor returns the specified obstacle neighbor of the specified agent.
func (rvo *RVOSimulator) GetAgentObstacleNeighbor(agentNo, neighborNo uint16) int {
	agent := rvo.Agents[agentNo]
	obstacleNeighbor := agent.ObstacleNeighbors[neighborNo]
	return obstacleNeighbor.Obstacle.ID
}

// GetAgentORCALine returns the specified ORCA constraint of the specified agent.
func (rvo *RVOSimulator) GetAgentORCALine(agentNo, lineNo uint16) *Line {
	agent := rvo.Agents[agentNo]
	return agent.OrcaLines[lineNo]
}

// GetAgentPosition returns the two-dimensional position of a specified agent.
func (rvo *RVOSimulator) GetAgentPosition(agentNo uint16) *Vector2 {
	agent := rvo.Agents[agentNo]
	return agent.Position
}

// GetAgentGoal :
func (rvo *RVOSimulator) GetAgentGoal(agentNo uint16) *Vector2 {
	agent := rvo.Agents[agentNo]
	return agent.Goal
}

// GetAgentPrefVelocity returns the two-dimensional preferred velocity of a specified agent.
func (rvo *RVOSimulator) GetAgentPrefVelocity(agentNo uint16) *Vector2 {
	agent := rvo.Agents[agentNo]
	return agent.PrefVelocity
}

// GetAgentRadius returns the radius of a specified agent.
func (rvo *RVOSimulator) GetAgentRadius(agentNo uint16) float32 {
	agent := rvo.Agents[agentNo]
	return agent.Radius
}

// GetAgentTimeHorizon returns the time horizon of a specified agent.
func (rvo *RVOSimulator) GetAgentTimeHorizon(agentNo uint16) float32 {
	agent := rvo.Agents[agentNo]
	return agent.TimeHorizon
}

// GetAgentTimeHorizonObst  returns the time horizon with respect to obstacles of a specified agent.
func (rvo *RVOSimulator) GetAgentTimeHorizonObst(agentNo uint16) float32 {
	agent := rvo.Agents[agentNo]
	return agent.TimeHorizonObst
}

// GetAgentVelocity returns the two-dimensional linear velocity of a specified agent.
func (rvo *RVOSimulator) GetAgentVelocity(agentNo uint16) *Vector2 {
	agent := rvo.Agents[agentNo]
	return agent.Velocity
}

// GetGlobalTime returns the global time of the simulation.
func (rvo *RVOSimulator) GetGlobalTime() float32 {
	return rvo.GlobalTime
}

// GetNumAgents returns the count of agents in the simulation.
func (rvo *RVOSimulator) GetNumAgents() uint16 {
	return uint16(len(rvo.Agents))
}

// GetNumObstacleVertices returns the count of obstacle vertices in the simulation.
func (rvo *RVOSimulator) GetNumObstacleVertices() int {
	return len(rvo.ObstacleVertices)
}

// GetObstacleVertex returns the two-dimensional position of a specified obstacle vertex.
func (rvo *RVOSimulator) GetObstacleVertex(vertexNo int) *Vector2 {
	obstacle := rvo.ObstacleVertices[vertexNo]
	return obstacle.Point
}

// GetObstacles returns list of all obstacles.
func (rvo *RVOSimulator) GetObstacles() [][]*Vector2 {
	return rvo.Obstacles
}

// GetNumObstacles returns the count of obstacles.
func (rvo *RVOSimulator) GetNumObstacles() int {
	return len(rvo.Obstacles)
}

// GetObstacle returns specific obstacle by id
func (rvo *RVOSimulator) GetObstacle(obstacleNo int) []*Vector2 {
	return rvo.Obstacles[obstacleNo]
}

// GetNextObstacleVertexNo returns the number of the obstacle vertex succeeding the specified obstacle vertex in its polygon.
func (rvo *RVOSimulator) GetNextObstacleVertexNo(vertexNo int) int {
	obstacle := rvo.ObstacleVertices[vertexNo]
	return obstacle.NextObstacle.ID
}

// GetPrevObstacleVertexNo returns the number of the obstacle vertex preceding the
// specified obstacle vertex in its polygon.
func (rvo *RVOSimulator) GetPrevObstacleVertexNo(vertexNo int) int {
	obstacle := rvo.ObstacleVertices[vertexNo]
	return obstacle.PrevObstacle.ID
}

// GetTimeStep returns the time step of the simulation.
func (rvo *RVOSimulator) GetTimeStep() float32 {
	return rvo.TimeStep
}

// ProcessObstacles processes the obstacles that have been added so that they are
// accounted for in the simulation.
func (rvo *RVOSimulator) ProcessObstacles() {
	rvo.KdTree.BuildObstacleTree()
}

// QueryVisibility performs a visibility query between the two specified points with respect to the obstacles.
func (rvo *RVOSimulator) QueryVisibility(point1 *Vector2, point2 *Vector2, radius float32) bool {
	return rvo.KdTree.QueryVisibility(point1, point2, radius)
}

// SetAgentDefaults sets the default properties for any new agent that is added.
func (rvo *RVOSimulator) SetAgentDefaults(neighborDist float32, maxNeighbors uint16, timeHorizon, timeHorizonObst, radius, maxSpeed float32, velocity *Vector2) {
	if rvo.DefaultAgent == nil {
		rvo.DefaultAgent = NewEmptyAgent()
	}

	rvo.DefaultAgent.MaxNeighbors = maxNeighbors
	rvo.DefaultAgent.MaxSpeed = maxSpeed
	rvo.DefaultAgent.NeighborDist = neighborDist
	rvo.DefaultAgent.Radius = radius
	rvo.DefaultAgent.TimeHorizon = timeHorizon
	rvo.DefaultAgent.TimeHorizonObst = timeHorizonObst
	rvo.DefaultAgent.Velocity = velocity

}

// SetAgentMaxNeighbors sets the maximum neighbor count of a specified agent.
func (rvo *RVOSimulator) SetAgentMaxNeighbors(agentNo, maxNeighbors uint16) {
	rvo.Agents[agentNo].MaxNeighbors = maxNeighbors
}

// SetAgentMaxSpeed sets the maximum speed of a specified agent.
func (rvo *RVOSimulator) SetAgentMaxSpeed(agentNo uint16, maxSpeed float32) {
	rvo.Agents[agentNo].MaxSpeed = maxSpeed
}

// SetAgentNeighborDist sets the maximum neighbor distance of a specified agent.
func (rvo *RVOSimulator) SetAgentNeighborDist(agentNo uint16, neighborDist float32) {
	rvo.Agents[agentNo].NeighborDist = neighborDist
}

// SetAgentPosition sets the two-dimensional position of a specified agent.
func (rvo *RVOSimulator) SetAgentPosition(agentNo uint16, position *Vector2) {
	rvo.Agents[agentNo].Position = position
}

// SetAgentGoal sets the two-dimensional goal of a specified agent.
func (rvo *RVOSimulator) SetAgentGoal(agentNo uint16, goal *Vector2) {
	rvo.Agents[agentNo].Goal = goal
}

// SetAgentPrefVelocity sets the two-dimensional preferred velocity of a specified agent.
func (rvo *RVOSimulator) SetAgentPrefVelocity(agentNo uint16, prefVelocity *Vector2) {
	rvo.Agents[agentNo].PrefVelocity = prefVelocity
}

// SetAgentRadius sets the radius of a specified agent.
func (rvo *RVOSimulator) SetAgentRadius(agentNo uint16, radius float32) {
	rvo.Agents[agentNo].Radius = radius
}

// SetAgentTimeHorizon sets the time horizon of a specified agent with respect to other agents.
func (rvo *RVOSimulator) SetAgentTimeHorizon(agentNo uint16, timeHorizon float32) {
	rvo.Agents[agentNo].TimeHorizon = timeHorizon
}

// SetAgentTimeHorizonObst sets the time horizon of a specified agent with respect to obstacles.
func (rvo *RVOSimulator) SetAgentTimeHorizonObst(agentNo uint16, timeHorizonObst float32) {
	rvo.Agents[agentNo].TimeHorizonObst = timeHorizonObst
}

// SetAgentVelocity sets the two-dimensional linear velocity of a specified agent.
func (rvo *RVOSimulator) SetAgentVelocity(agentNo uint16, velocity *Vector2) {
	rvo.Agents[agentNo].Velocity = velocity
}

// SetTimeStep sets the time step of the simulation.
func (rvo *RVOSimulator) SetTimeStep(timeStep float32) {
	rvo.TimeStep = timeStep
}
