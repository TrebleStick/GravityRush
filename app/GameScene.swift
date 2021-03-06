//
//  GameScene.swift
//  Gravity Rush
//
//  Created by Ryan Armiger on 29/01/2019.
//  Copyright © 2019 Ryan Armiger. All rights reserved.
//

import SpriteKit
//import GameKit

class GameScene: SKScene {
  
    
    private var ship: SKSpriteNode = SKSpriteNode()
//    private var score: SKLabelNode = SKLabelNode()
    private var score: SKLabelNode?
    private var cam: SKCameraNode?
//    private var rotationVal: CGFloat = 0
//    private var offset: CGFloat = 0
    private let rotateRecogniser = UIRotationGestureRecognizer()
    private let tapRecogniser = UITapGestureRecognizer()
    
 
    override func sceneDidLoad() {
//        addEmitter()
    }
    
    override func didMove(to view: SKView) {
//        super.didMove(to: view)
        
        //Setup camera
        cam = SKCameraNode()
        self.camera = cam
        self.addChild(cam!)
        
        //Setup ship
        if let sprite:SKSpriteNode = self.childNode(withName: "Ship") as? SKSpriteNode {
            ship = sprite
        } else { print("ERROR: Ship not initiated") }
//        ship.constraints?.append(SKConstraint.positionX(SKRange(lowerLimit: -(self.size.width / 2) + (ship.size.width / 2), upperLimit: (self.size.width / 2) - (ship.size.width / 2))))
        
        //Setup score and add to camera
        score = SKLabelNode(fontNamed: "Helvetica Neue UltraLight")
//        score?.fontSize = 48
        score?.text = "1000"
        score?.color = UIColor.white
        score?.position = CGPoint(x: self.size.width / 2 - 75, y:      self.size.height / 2 - 75)
        self.camera?.addChild(score!)
        
        
        //Set up actions for rotation and tapping
        rotateRecogniser.addTarget(self, action: #selector(GameScene.rotatedView(_:)))
        tapRecogniser.addTarget(self, action: #selector(GameScene.tapView))
        tapRecogniser.numberOfTapsRequired = 1
        tapRecogniser.numberOfTouchesRequired = 1
        self.view?.addGestureRecognizer(rotateRecogniser)
        self.view?.addGestureRecognizer(tapRecogniser)
    }

    
    override func update(_ currentTime: TimeInterval) {
        
        keepPlayerInBounds()
      
        if let camera = cam {
            camera.position.y = ship.position.y
            camera.position.x = 0
        }
        
//        rotationVal = ship.zRotation

        if Int(arc4random()) % 4 == 0 {
            if let intScore = Int((self.score?.text!)!) {
                self.score?.text = String(intScore + 15)
            }
        }
    }
    
    private func keepPlayerInBounds() {
        if ship.position.x > (self.size.width / 2) - (ship.size.width / 2) {
            ship.position.x = (self.size.width / 2) - (ship.size.width / 2)
        }
        if ship.position.x <  -(self.size.width / 2) + (ship.size.width / 2) {
            ship.position.x = -(self.size.width / 2) + (ship.size.width / 2)
        }
    }
    
    

    @objc func tapView(){
//        print("Tapped")
        
        let xVec: CGFloat = sin(ship.zRotation) * -1
        let yVec: CGFloat = cos(ship.zRotation) * 1
        
        ship.physicsBody?.applyImpulse(CGVector(dx: xVec, dy: yVec))

    }
    
    @objc func rotatedView(_ sender: UIRotationGestureRecognizer){
        if sender.state == .began {
//            print("Gesture began")
        }
        if sender.state == .changed {

            if sender.rotation.isLessThanOrEqualTo(0) {
                ship.zRotation += 0.1
            } else {
                ship.zRotation -= 0.1
            }
        }
        if sender.state == .ended {
        }
    }
    
    func touchDown(atPoint pos : CGPoint) {

    }
    
    func touchMoved(toPoint pos : CGPoint) {

    }
    
    func touchUp(atPoint pos : CGPoint) {

    }
    
//    func addEmitter(){
//        guard let emitter = SKEmitterNode(fileNamed: "Magic") else {
//            return
//        }
//        emitter.zPosition = 2
//        emitter.position = CGPoint(x: size.width, y: size.height)
//        addChild(emitter)
//    }
    
    override func touchesBegan(_ touches: Set<UITouch>, with event: UIEvent?) {
        for t in touches { self.touchDown(atPoint: t.location(in: self)) }
    }
    
    override func touchesMoved(_ touches: Set<UITouch>, with event: UIEvent?) {
        for t in touches { self.touchMoved(toPoint: t.location(in: self)) }
    }
    
    override func touchesEnded(_ touches: Set<UITouch>, with event: UIEvent?) {
        for t in touches { self.touchUp(atPoint: t.location(in: self)) }
    }
    
    override func touchesCancelled(_ touches: Set<UITouch>, with event: UIEvent?) {
        for t in touches { self.touchUp(atPoint: t.location(in: self)) }
    }
    
    
}
