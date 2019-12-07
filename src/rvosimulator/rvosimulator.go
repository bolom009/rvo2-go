package rvosimulator

var (
	Sim *RVOSimulator
)

func init() {
}

// RVOSimulator :
type RVOSimulator struct {
	TimeStep     float64
	Agents       []*Agent
	Obstacles    []*Obstacle
	KdTree       *KdTree
	DefaultAgent *Agent
	GlobalTime   float64
}

// NewRVOSimulator : RVOSimulator with options
func NewRVOSimulator(timeStep float64, neighborDist float64, maxNeighbors int, timeHorizon float64, timeHorizonObst float64, radius float64, maxSpeed float64, velocity *Vector2) *RVOSimulator {
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
		TimeStep:     timeStep,
		Agents:       make([]*Agent, 0),
		Obstacles:    make([]*Obstacle, 0),
		KdTree:       kdTree,
		DefaultAgent: defaultAgent,
		GlobalTime:   0.0,
	}
	Sim = sim

	return sim
}

// NewDefaultRVOSimulator : RVOSimulator with default values
func NewEmptyRVOSimulator() *RVOSimulator {
	kdTree := NewKdTree()
	defaultAgent := NewEmptyAgent()

	sim := &RVOSimulator{
		TimeStep:     0,
		Agents:       make([]*Agent, 0),
		Obstacles:    make([]*Obstacle, 0),
		KdTree:       kdTree,
		DefaultAgent: defaultAgent,
		GlobalTime:   0.0,
	}
	Sim = sim
	return sim
}

type AddAgentParam struct {
	Position        *Vector2
	NeighborDist    float64
	MaxNeighbors    int
	TimeHorizon     float64
	TimeHorizonObst float64
	Radius          float64
	MaxSpeed        float64
	Velocity        *Vector2
}

// AddDefaultAgent : Add agent with default values
func (rvo *RVOSimulator) AddDefaultAgent(position *Vector2) (int, bool) {

	if rvo.DefaultAgent == nil {
		err := true
		return -1, err
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
	agent.ID = len(rvo.Agents)

	rvo.Agents = append(rvo.Agents, agent)

	return len(rvo.Agents) - 1, false
}

// AddAgent : Add agent with options
func (rvo *RVOSimulator) AddAgent(position *Vector2, neighborDist float64, maxNeighbors int, timeHorizon float64, timeHorizonObst float64, radius float64, maxSpeed float64, velocity *Vector2) (int, bool) {

	agent := NewEmptyAgent()
	agent.Position = position
	agent.MaxNeighbors = maxNeighbors
	agent.MaxSpeed = maxSpeed
	agent.NeighborDist = neighborDist
	agent.Radius = radius
	agent.TimeHorizon = timeHorizon
	agent.TimeHorizonObst = timeHorizonObst
	agent.Velocity = velocity
	agent.ID = len(rvo.Agents)

	rvo.Agents = append(rvo.Agents, agent)

	return len(rvo.Agents) - 1, false
}

// AddObstacle : Add Obstacle with vertices
func (rvo *RVOSimulator) AddObstacle(vertices []*Vector2) (int, bool) {

	if len(vertices) < 2 {
		err := true
		return -1, err
	}

	// 一つ一つ大きなObstacleはObstacleNoとして管理
	obstacleNo := len(rvo.Obstacles)

	// Obstacleを一点ずつ置いて行って形を作る
	for i := 0; i < len(vertices); i++ {
		obstacle := NewEmptyObstacle()
		obstacle.Point = vertices[i]

		// NextとPrevObstacleをセット
		if i != 0 {
			obstacle.PrevObstacle = rvo.Obstacles[len(rvo.Obstacles)-1]
			obstacle.PrevObstacle.NextObstacle = obstacle
		}

		if i == len(vertices)-1 {
			obstacle.NextObstacle = rvo.Obstacles[obstacleNo]
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
			obstacle.IsConvex = (LeftOf(vertices[ki], vertices[i], vertices[ti]) >= 0.0)
		}

		obstacle.ID = len(rvo.Obstacles)

		rvo.Obstacles = append(rvo.Obstacles, obstacle)

	}

	return obstacleNo, false
}

// DoStep : Forward Step
func (rvo *RVOSimulator) DoStep() {
	rvo.KdTree.BuildAgentTree()

	for i := 0; i < len(rvo.Agents); i++ {
		// agentのneighborsを計算
		rvo.Agents[i].ComputeNeighbors()
		// agentの速度を計算
		rvo.Agents[i].ComputeNewVelocity()
	}

	for i := 0; i < len(rvo.Agents); i++ {
		// agentを更新
		rvo.Agents[i].Update()
	}

	// globaltimeを更新
	rvo.GlobalTime += rvo.TimeStep
}

// GetAgentAgentNeighbor :
func (rvo *RVOSimulator) GetAgentAgentNeighbor(agentNo int, neighborNo int) int {
	return rvo.Agents[agentNo].AgentNeighbors[neighborNo].Agent.ID
}

// GetAgentMaxNeighbors :
func (rvo *RVOSimulator) GetAgentMaxNeighbors(agentNo int) int {
	agent := rvo.Agents[agentNo]
	return agent.MaxNeighbors
}

// GetAgentMaxSpeed :
func (rvo *RVOSimulator) GetAgentMaxSpeed(agentNo int) float64 {
	agent := rvo.Agents[agentNo]
	return agent.MaxSpeed
}

// GetAgentNeighborDist :
func (rvo *RVOSimulator) GetAgentNeighborDist(agentNo int) float64 {
	agent := rvo.Agents[agentNo]
	return agent.NeighborDist
}

// GetAgentNumAgentNeighbors :
func (rvo *RVOSimulator) GetAgentNumAgentNeighbors(agentNo int) int {
	agent := rvo.Agents[agentNo]
	return len(agent.AgentNeighbors)
}

// GetAgentNumObstacleNeighbors :
func (rvo *RVOSimulator) GetAgentNumObstacleNeighbors(agentNo int) int {
	agent := rvo.Agents[agentNo]
	return len(agent.ObstacleNeighbors)
}

// GetAgentNumORCALines :
func (rvo *RVOSimulator) GetAgentNumORCALines(agentNo int) int {
	agent := rvo.Agents[agentNo]
	return len(agent.OrcaLines)
}

// GetAgentObstacleNeighbor :
func (rvo *RVOSimulator) GetAgentObstacleNeighbor(agentNo int, neighborNo int) int {
	agent := rvo.Agents[agentNo]
	obstacleNeighbor := agent.ObstacleNeighbors[neighborNo]
	return obstacleNeighbor.Obstacle.ID
}

// GetAgentORCALine :
func (rvo *RVOSimulator) GetAgentORCALine(agentNo int, lineNo int) *Line {
	agent := rvo.Agents[agentNo]
	return agent.OrcaLines[lineNo]
}

// GetAgentPosition :
func (rvo *RVOSimulator) GetAgentPosition(agentNo int) *Vector2 {
	agent := rvo.Agents[agentNo]
	return agent.Position
}

// GetAgentPrefVelocity :
func (rvo *RVOSimulator) GetAgentPrefVelocity(agentNo int) *Vector2 {
	agent := rvo.Agents[agentNo]
	return agent.PrefVelocity
}

// GetAgentRadius :
func (rvo *RVOSimulator) GetAgentRadius(agentNo int) float64 {
	agent := rvo.Agents[agentNo]
	return agent.Radius
}

// GetAgentTimeHorizon :
func (rvo *RVOSimulator) GetAgentTimeHorizon(agentNo int) float64 {
	agent := rvo.Agents[agentNo]
	return agent.TimeHorizon
}

// GetAgentTimeHorizonObst :
func (rvo *RVOSimulator) GetAgentTimeHorizonObst(agentNo int) float64 {
	agent := rvo.Agents[agentNo]
	return agent.TimeHorizonObst
}

// GetAgentVelocity :
func (rvo *RVOSimulator) GetAgentVelocity(agentNo int) *Vector2 {
	agent := rvo.Agents[agentNo]
	return agent.Velocity
}

// GetGlobalTime :
func (rvo *RVOSimulator) GetGlobalTime() float64 {
	return rvo.GlobalTime
}

// GetNumAgents :
func (rvo *RVOSimulator) GetNumAgents() int {
	return len(rvo.Agents)
}

// GetNumObstacleVertices :
func (rvo *RVOSimulator) GetNumObstacleVertices() int {
	return len(rvo.Obstacles)
}

// GetObstacleVertex :
func (rvo *RVOSimulator) GetObstacleVertex(vertexNo int) *Vector2 {
	obstacle := rvo.Obstacles[vertexNo]
	return obstacle.Point
}

// GetNextObstacleVertexNo :
func (rvo *RVOSimulator) GetNextObstacleVertexNo(vertexNo int) int {
	obstacle := rvo.Obstacles[vertexNo]
	return obstacle.NextObstacle.ID
}

// GetPrevObstacleVertexNo :
func (rvo *RVOSimulator) GetPrevObstacleVertexNo(vertexNo int) int {
	obstacle := rvo.Obstacles[vertexNo]
	return obstacle.PrevObstacle.ID
}

// GetTimeStep :
func (rvo *RVOSimulator) GetTimeStep() float64 {
	return rvo.TimeStep
}

// ProcessObstacles :
func (rvo *RVOSimulator) ProcessObstacles() {
	rvo.KdTree.BuildObstacleTree()
}

// QueryVisibility :
func (rvo *RVOSimulator) QueryVisibility(point1 *Vector2, point2 *Vector2, radius float64) bool {
	return rvo.KdTree.QueryVisibility(point1, point2, radius)
}

// SetAgentDefaults :
func (rvo *RVOSimulator) SetAgentDefaults(neighborDist float64, maxNeighbors int, timeHorizon float64, timeHorizonObst float64, radius float64, maxSpeed float64, velocity *Vector2) {
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

// SetAgentMaxNeighbors :
func (rvo *RVOSimulator) SetAgentMaxNeighbors(agentNo int, maxNeighbors int) {
	rvo.Agents[agentNo].MaxNeighbors = maxNeighbors
}

// SetAgentMaxSpeed :
func (rvo *RVOSimulator) SetAgentMaxSpeed(agentNo int, maxSpeed float64) {
	rvo.Agents[agentNo].MaxSpeed = maxSpeed
}

// SetAgentNeighborDist :
func (rvo *RVOSimulator) SetAgentNeighborDist(agentNo int, neighborDist float64) {
	rvo.Agents[agentNo].NeighborDist = neighborDist
}

// SetAgentPosition :
func (rvo *RVOSimulator) SetAgentPosition(agentNo int, position *Vector2) {
	rvo.Agents[agentNo].Position = position
}

// CHECK OK
// SetAgentPrefVelocity :
func (rvo *RVOSimulator) SetAgentPrefVelocity(agentNo int, prefVelocity *Vector2) {
	rvo.Agents[agentNo].PrefVelocity = prefVelocity

}

// SetAgentRadius :
func (rvo *RVOSimulator) SetAgentRadius(agentNo int, radius float64) {
	rvo.Agents[agentNo].Radius = radius
}

// SetAgentTimeHorizon :
func (rvo *RVOSimulator) SetAgentTimeHorizon(agentNo int, timeHorizon float64) {
	rvo.Agents[agentNo].TimeHorizon = timeHorizon
}

// SetAgentTimeHorizonObst :
func (rvo *RVOSimulator) SetAgentTimeHorizonObst(agentNo int, timeHorizonObst float64) {
	rvo.Agents[agentNo].TimeHorizonObst = timeHorizonObst
}

// SetAgentVelocity :
func (rvo *RVOSimulator) SetAgentVelocity(agentNo int, velocity *Vector2) {
	rvo.Agents[agentNo].Velocity = velocity
}

// SetTimeStep :
func (rvo *RVOSimulator) SetTimeStep(timeStep float64) {
	rvo.TimeStep = timeStep
}
