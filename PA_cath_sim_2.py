import Sofa
import sys
import numpy as np
import glob

files = (glob.glob("./output_data/sim_2/position/*"))

print(files)
if files ==[]:
	i=1
else:
	i = len(files)+1


class tip_position(Sofa.Core.Controller):
	
	def __init__(self, *args, **kwargs):
		Sofa.Core.Controller.__init__(self, *args, **kwargs)
		self.iteration = 0
	
	def onKeypressedEvent(self,event):
	
		root = self.getContext()
		time = root.time.value

		if event['key']=='\x13':
			key = 1
		elif event['key']=='\x15':
			key = 2
		elif event['key']=='\x14':
			key = 3
		elif event['key']=='\x12':
			key = 4
		else:
			key=0

		if key==1 or key==2 or key==3 or key==4:
			with open(f'./output_data/sim_2/key_press/key_press_{i}.txt', 'a') as f:
				f.write(str(key)+','+str(time)+'\n')

		return key

	def onAnimateEndEvent(self,event):

		root = self.getContext()
		myMOpositions  = root.getObject('DOFs Container').findData('position').value
		time = root.time.value
		with open(f'./output_data/sim_2/position/position_{i}.txt', 'a') as f:
			f.write(str(myMOpositions[-1][0])+','+str(myMOpositions[-1][1])+','+str(myMOpositions[-1][2])+','+str(myMOpositions[-1][3])+','+str(myMOpositions[-1][4])+','+str(myMOpositions[-1][5])+','+str(myMOpositions[-1][6])+','+str(time)+'\n')

		return 0

def createScene(rootNode):

	rootNode.gravity = [0,-9.81,0]
	rootNode.dt = 0.01
	rootNode.bbox = "0 0 0 20 20 20"

	rootNode.addObject('RequiredPlugin', pluginName='BeamAdapter SofaMeshCollision SofaBoundaryCondition SofaConstraint SofaMiscCollision SofaDeformable SofaGeneralLinearSolver SofaImplicitOdeSolver Sofa.Component.Collision.Detection.Algorithm Sofa.Component.IO.Mesh Sofa.Component.SolidMechanics.FEM.Elastic Sofa.GL.Component.Rendering3D')
	rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels hideBehaviorForceField hideCollisionModels hideMappings hideInteractionForceFields')

	rootNode.addObject('FreeMotionAnimationLoop')
	rootNode.addObject('DefaultVisualManagerLoop')

	rootNode.addObject('LCPConstraintSolver', mu='0.1', tolerance='1e-10', maxIt='1000', build_lcp='false')
	rootNode.addObject('DefaultPipeline', draw='0', depth='6', verbose='1')
	rootNode.addObject('BruteForceBroadPhase', name='N2')
	rootNode.addObject('BVHNarrowPhase')
	rootNode.addObject('LocalMinDistance', contactDistance='1', alarmDistance='5', name='localmindistance', angleCone='0.02')
	rootNode.addObject('CollisionResponse', name='Response', response='FrictionContactConstraint',responseParams='mu=10.0')
	rootNode.addObject('BackgroundSetting', color ='0.5 0.6 0.7')

	topoLines = rootNode.addChild('EdgeTopology')
	topoLines.addObject('WireRestShape', name='BeamRestShape', 
								 straightLength=940.0, length=1000.0, 
								 numEdges=200, youngModulus=1e4, 
								 spireDiameter=75, numEdgesCollis=[50,10], 
								 printLog=True, template='Rigid3d', spireHeight=0.0, 
								 densityOfBeams=[45,7], youngModulusExtremity=1e4)
	topoLines.addObject('EdgeSetTopologyContainer', name='meshLinesBeam')
	topoLines.addObject('EdgeSetTopologyModifier', name='Modifier')
	topoLines.addObject('EdgeSetGeometryAlgorithms', name='GeomAlgo', template='Rigid3d')
	topoLines.addObject('MechanicalObject', name='dofTopo2', template='Rigid3d')


	BeamMechanics = rootNode.addChild('BeamModel')
	BeamMechanics.addObject('EulerImplicitSolver', rayleighStiffness=0.1, rayleighMass=0.1)
	BeamMechanics.addObject('BTDLinearSolver', verbose=False)
	BeamMechanics.addObject('RegularGridTopology', name='MeshLines', 
									nx=60, ny=1, nz=1,
									xmax=0.0, xmin=0.0, ymin=0, ymax=0, zmax=0, zmin=0,
									p0=[0,0,0], drawEdges=False)
	BeamMechanics.addObject('MechanicalObject', name='DOFs Container', template='Rigid3d', ry=-90, scale=0.5)
	BeamMechanics.addObject('WireBeamInterpolation', name='BeamInterpolation', WireRestShape='@../EdgeTopology/BeamRestShape', 
									radius=0.9, printLog=False)
	BeamMechanics.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', massDensity=0.00000155, interpolation='@BeamInterpolation')
	BeamMechanics.addObject('InterventionalRadiologyController', name='DeployController', template='Rigid3d', instruments='BeamInterpolation', 
									startingPos=[0, 0, 0, 0, 0, 0, 1], xtip=[0, 0, 0], printLog=True, 
								   rotationInstrument=[0, 90, 0], step=5, speed=0, 
									listening=1, controlledInstrument=0,angularStep=2*np.pi/36)
	BeamMechanics.addObject('LinearSolverConstraintCorrection', wire_optimization='true', printLog=0)
	BeamMechanics.addObject('FixedConstraint', indices=0, name='FixedConstraint')
	BeamMechanics.addObject('RestShapeSpringsForceField', name="RestSPForceField", points='@DeployController.indexFirstNode', angularStiffness=1e5, stiffness=1e5)
	#BeamMechanics.addObject('BeamFEMForceField',name="FEM" ,radius="0.1", youngModulus="20000000" ,poissonRatio="0.49")
	BeamMechanics.addObject(tip_position(name='test'))

	BeamCollis = BeamMechanics.addChild('CollisionModel')
	BeamCollis.activated = True
	BeamCollis.addObject('EdgeSetTopologyContainer', name='collisEdgeSet')
	BeamCollis.addObject('EdgeSetTopologyModifier', name='colliseEdgeModifier')
	BeamCollis.addObject('MechanicalObject', name='CollisionDOFs')
	BeamCollis.addObject('MultiAdaptiveBeamMapping', controller='../DeployController', useCurvAbs=True, printLog=False, name='collisMap')
	BeamCollis.addObject('LineCollisionModel', proximity=0.0)
	BeamCollis.addObject('PointCollisionModel', proximity=0.0)
	BeamCollis.addObject('GenericConstraintSolver', maxIt='10000', tolerance='1e-7')
	BeamCollis.addObject('UncoupledConstraintCorrection')

	Vessel = rootNode.addChild('Vessel')
	Vessel.addObject('EulerImplicitSolver', name="ImplicitEulerSolver", rayleighMass=0.1, rayleighStiffness=0.1)
	Vessel.addObject('CGLinearSolver', name="CGLinearSolver", iterations=25, tolerance=1e-9, threshold=1e-9)
	Vessel.addObject('SparseGridTopology', fileTopology="/home/mark/Documents/SOFA/PA_catheterization_simulation/mesh/First Model - Rishi.obj", n="20 6 6")
	Vessel.addObject('MechanicalObject', name="VesselState", scale=1, translation=[150,110,280], rotation=[-90 ,0,180])
	Vessel.addObject('UniformMass', totalMass=0.05)
	Vessel.addObject('HexahedronFEMForceField', template="Vec3d", name="FEM" ,poissonRatio=0.4, youngModulus=1000)
	Vessel.addObject('FixedConstraint', indices=[0, 1, 2, 3, 4, 7, 8])
	Vessel.addObject('GenericConstraintSolver', maxIt='10000', tolerance='1e-7')
	Vessel.addObject('UncoupledConstraintCorrection')

	CollisionModel = Vessel.addChild('CollisionModel')
	CollisionModel.addObject('MeshOBJLoader', name="loader", filename='/home/mark/Documents/SOFA/PA_catheterization_simulation/mesh/First Model - Rishi.obj')

	CollisionModel.addObject('TriangleSetTopologyContainer',name="Container" ,src="@loader")
	CollisionModel.addObject('TriangleSetTopologyModifier',name="Modifier")
	CollisionModel.addObject('TriangleSetGeometryAlgorithms',name="GeomAlgo",template="Vec3d")
	CollisionModel.addObject('MechanicalObject',name="CollisionState",src="@loader")
	CollisionModel.addObject('TriangleCollisionModel', template="Vec3d", name="Triangles", contactStiffness=1, bothSide=1)
	CollisionModel.addObject('LineCollisionModel', template="Vec3d", name="Lines", contactStiffness="@[-1].contactStiffness",bothSide=1)
	CollisionModel.addObject('PointCollisionModel', template="Vec3d", name="Points", contactStiffness="@[-1].contactStiffness",bothSide=1)
	CollisionModel.addObject('BarycentricMapping')
 
	VisualModel = CollisionModel.addChild('VisualModel')
	VisualModel.addObject('OglModel', name="Visual", material="Default Diffuse 1 0 0 1 0.3 Ambient 1 0 0 0.1 0.3")
	VisualModel.addObject('IdentityMapping', input="@..", output="@Visual" )

def main():
	import SofaRuntime
	import Sofa.Gui

	root = Sofa.Core.Node('root')
	createScene(root)
	Sofa.Simulation.init(root)

	Sofa.Gui.GUIManager.Init('myscene', 'qglviewer')
	Sofa.Gui.GUIManager.createGUI(root, __file__)
	Sofa.Gui.GUIManager.SetDimension(1920, 1080)
	Sofa.Gui.GUIManager.MainLoop(root)
	Sofa.Gui.GUIManager.closeGUI()


# Function used only if this script is called from a python environment
if __name__ == '__main__':
	main()
