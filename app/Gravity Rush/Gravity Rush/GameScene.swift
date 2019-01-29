//
//  GameScene.swift
//  Gravity Rush
//
//  Created by Ryan Armiger on 29/01/2019.
//  Copyright © 2019 Ryan Armiger. All rights reserved.
//

import SpriteKit
import GameplayKit

class GameScene: SKScene {
    
    var entities = [GKEntity]()
    var graphs = [String : GKGraph]()
    
    var ship: SKSpriteNode = SKSpriteNode()
    var score: SKLabelNode = SKLabelNode()
    
    private var lastUpdateTime : TimeInterval = 0
    private var label : SKLabelNode?
    private var spinnyNode : SKShapeNode?
    
    override func sceneDidLoad() {
//        self.anchorPoint = CGPoint(x: 0, y: 0)
        self.lastUpdateTime = 0
        addEmitter()
        
        if let sprite:SKSpriteNode = self.childNode(withName: "ship") as? SKSpriteNode {
            ship = sprite
        }
        if let label:SKLabelNode = self.childNode(withName: "score") as? SKLabelNode {
            score = label
        }
    }
    
    override func didMove(to view: SKView) {
//        addEmitter()
    }
    
    
    func touchDown(atPoint pos : CGPoint) {

    }
    
    func touchMoved(toPoint pos : CGPoint) {

    }
    
    func touchUp(atPoint pos : CGPoint) {

    }
    
    func addEmitter(){
        guard let emitter = SKEmitterNode(fileNamed: "Magic") else {
            return
        }
        emitter.zPosition = 2
        emitter.position = CGPoint(x: size.width, y: size.height)
        addChild(emitter)
    }
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
    
    
    override func update(_ currentTime: TimeInterval) {
        // Called before each frame is rendered
        
        
        
        // Initialize _lastUpdateTime if it has not already been
        if (self.lastUpdateTime == 0) {
            self.lastUpdateTime = currentTime
            self.score.text = "1000"
        }
        
        
        // Calculate time since last update
        let dt = currentTime - self.lastUpdateTime
        
//        if ship.position.y < 0 {
        if Int(arc4random()) % 4 == 0 {
            if let intScore = Int(self.score.text!) {
                self.score.text = String(intScore + 15)
            }
        }
        
//        }
        
        // Update entities
        for entity in self.entities {
            entity.update(deltaTime: dt)
        }
        
        self.lastUpdateTime = currentTime
    }
}
