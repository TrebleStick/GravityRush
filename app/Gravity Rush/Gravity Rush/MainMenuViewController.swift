//
//  MainMenuViewController.swift
//  Gravity Rush
//
//  Created by Ryan Armiger on 18/02/2019.
//  Copyright © 2019 Ryan Armiger. All rights reserved.
//

import UIKit
import SpriteKit
import GameplayKit
import CoreBluetooth

let deviceServiceCBUUID = CBUUID(string: "0x180D")

//class MainMenuViewController: UIViewController, CBCentralManagerDelegate, CBPeripheralDelegate {
class MainMenuViewController: UIViewController{

    var centralManager: CBCentralManager!
    var myoPeripheral: CBPeripheral!
//    var device: CBPeripheral?
//
//    let serviceUUID = CBUUID(string: "1812")
//    let deviceCharacteristicUUID = CBUUID(string: <#T##String#>)
    
    @IBOutlet weak var gravityRushLabel: UILabel!
    @IBOutlet weak var leaderBoardContainer: UIView!
    @IBOutlet weak var settingsView: UIView!
    
    override func viewDidLoad() {
        super.viewDidLoad()
//        centralManager = CBCentralManager()
//        centralManager.delegate = self
        centralManager = CBCentralManager(delegate: self, queue: nil)
        
        leaderBoardContainer.isHidden = true
        settingsView.isHidden = true
        // Load 'GameScene.sks' as a GKScene. This provides gameplay related content
        // including entities and graphs.
        if let scene = GKScene(fileNamed: "MainMenuScene") {
            
            // Get the SKScene from the loaded GKScene
            if let sceneNode = scene.rootNode as! SKScene? {
                
                // Copy gameplay related content over to the scene
                //                sceneNode.entities = scene.entities
                //                sceneNode.graphs = scene.graphs
                //
                // Set the scale mode to scale to fit the window
                sceneNode.scaleMode = .aspectFill
                
                // Present the scene
                if let view = self.view as! SKView? {
                    view.presentScene(sceneNode)
                    
                    view.ignoresSiblingOrder = true
                    //                    view.showsDrawCount = true
                    //                    view.showsFPS = true
                    //                    view.showsNodeCount = true
                }
            }
        }
    }
    
    
//
//    func centralManagerDidUpdateState(_ central: CBCentralManager) {
//        if central.state == .poweredOn {
//            centralManager.scanForPeripherals(withServices: [serviceUUID], options: nil)
//        }
//    }
//    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
//        centralManager.stopScan()
//        device = peripheral
//        centralManager.connect(peripheral, options: nil)
//    }
//    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
//        peripheral.delegate = self
//        peripheral.discoverServices([serviceUUID])
//    }
//
//
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
////        if let characteristic = service.characteristics {
////            print(characteristic)
////        }
//        print("in didDiscoverServices")
//        if let service = peripheral.services {
//            print(service)
//        }
//        if let service = peripheral.services?.first(where: { $0.uuid == serviceUUID}) {
//            print("discovered services..")
//            print(service.characteristics)
////            peripheral.discoverCharacteristics([deviceCharacteristicUUID], for: service)
//        }
//    }
//
//
//    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
//        if let characteristic = service.characteristics {
//            print(characteristic)
//        }
////        if let characteristic = service.characteristics?.first(where: { $0.uuid == deviceCharacteristicUUID}) {
////            peripheral.setNotifyValue(true, for: characteristic)
////        }
//    }
//
//    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
//        if let data = characteristic.value {
//            print(data)
//        }
//    }
    
    override var shouldAutorotate: Bool {
        return true
    }
    
    override var supportedInterfaceOrientations: UIInterfaceOrientationMask {
        if UIDevice.current.userInterfaceIdiom == .phone {
            //            return .allButUpsideDown
            return .portrait
            
        } else {
            return .all
        }
    }
    
    override var prefersStatusBarHidden: Bool {
        return true
    }
    
    func toggleSettings(){
        if settingsView.isHidden == true{
            settingsView.isHidden = false
            gravityRushLabel.isHidden = true
            leaderBoardContainer.isHidden = true
        }
        else {
            gravityRushLabel.isHidden = false
            settingsView.isHidden = true
        }
    }
    
    func toggleLeaderboard(){
        if leaderBoardContainer.isHidden == true{
            leaderBoardContainer.isHidden = false
            gravityRushLabel.isHidden = true
            settingsView.isHidden = true
        }
        else {
            gravityRushLabel.isHidden = false
            leaderBoardContainer.isHidden = true
        }
    }
    @IBAction func LeaderboardButton(_ sender: Any) {
        toggleLeaderboard()
    }
    @IBAction func settingsButton(_ sender: Any) {
        toggleSettings()
    }
    @IBAction func playButton(_ sender: Any) {
        if let viewController:UIViewController = UIStoryboard(name: "Main", bundle: nil).instantiateViewController(withIdentifier: "game") as? GameViewController{
            self.present(viewController, animated: true, completion: nil)    }
        }
        // .instantiatViewControllerWithIdentifier() returns AnyObject! this must be downcast to utilize it
    
}

extension MainMenuViewController: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        switch central.state {
        case .unknown:
            print("central.state is .unknown")
        case .resetting:
            print("central.state is .resetting")
        case .unsupported:
            print("central.state is .unsupported")
        case .unauthorized:
            print("central.state is .unauthorized")
        case .poweredOff:
            print("central.state is .poweredOff")
        case .poweredOn:
            print("central.state is .poweredOn")
//            centralManager.scanForPeripherals(withServices: [deviceServiceCBUUID], options: nil)
            centralManager.scanForPeripherals(withServices: [deviceServiceCBUUID], options: nil)


        }
    }
    func centralManager(_ central: CBCentralManager, didDiscover peripheral: CBPeripheral, advertisementData: [String : Any], rssi RSSI: NSNumber) {
        print(peripheral)
        if peripheral.name == "GravityRush BLE"{
            print("saving GR BLE")
            myoPeripheral = peripheral
            myoPeripheral.delegate = self
            centralManager.stopScan()
            centralManager.connect(myoPeripheral)
        }
    }
    
    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        print("Connected to GR BLE")
        myoPeripheral.discoverServices(nil)
    }
}

extension MainMenuViewController: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        
        for service in services {
            print(service)
            peripheral.discoverCharacteristics(nil, for: service)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let characteristics = service.characteristics else { return }
        
        for characteristic in characteristics {
            print(characteristic)
        }
    }
}
