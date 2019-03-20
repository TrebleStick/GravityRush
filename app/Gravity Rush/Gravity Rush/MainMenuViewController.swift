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
import CoreML
import Accelerate

let deviceServiceCBUUID = CBUUID(string: "0x1812")
let HIDcharCBUUID = CBUUID(string: "6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
let HIDserviceCBUUID = CBUUID(string: "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")

//class MainMenuViewController: UIViewController, CBCentralManagerDelegate, CBPeripheralDelegate {
class MainMenuViewController: UIViewController{

    var centralManager: CBCentralManager!
    var myoPeripheral: CBPeripheral!
    var tempString: String = ""
    var dataArr: [[Double]] = []
    let alpha = bmodel()
    var flag = false
    private var actionWindow = [0,0]
    private var countWindow = 0
    private var action = [0,0,0,0]
    
    @IBOutlet weak var gravityRushLabel: UILabel!
    @IBOutlet weak var leaderBoardContainer: UIView!
    @IBOutlet weak var settingsView: UIView!
    
    override func viewDidLoad() {
        super.viewDidLoad()

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
            NotificationCenter.default.post(name: Notification.Name("reloadLeader"), object: nil)

            
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
    @IBAction func unwindToMenu(_ unwindSegue: UIStoryboardSegue) {
//        let sourceViewController = unwindSegue.source
        // Use data from the view controller which initiated the unwind segue

    }
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
        myoPeripheral.discoverServices([HIDserviceCBUUID])
    }
}

extension MainMenuViewController: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard let services = peripheral.services else { return }
        
        for service in services {
//            print(service)
            peripheral.discoverCharacteristics([HIDcharCBUUID], for: service)
        }
    }
    
    func peripheral(_ peripheral: CBPeripheral, didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard let characteristics = service.characteristics else { return }
        
        for characteristic in characteristics {
            print(characteristic)
            if characteristic.properties.contains(.read) {
                print("\(characteristic.uuid): properties contains .read")
            }
            if characteristic.properties.contains(.notify) {
                print("\(characteristic.uuid): properties contains .notify")
                peripheral.setNotifyValue(true, for: characteristic)
            }
//            peripheral.readValue(for: characteristic)
        }
    }
    func threadRecurse(){
        DispatchQueue.global(qos: .userInitiated).async { [weak self] in
            guard let self = self else {
                return
            }
            self.predictAction(localArr: self.dataArr)
            // 2
            DispatchQueue.main.async { [weak self] in
                // 3
                self?.countWindow += 1
                if self?.countWindow ?? 0 > 30 {
                    self?.startAction()
                }
                self?.threadRecurse()
            }
        }
    }
    
    func predictAction(localArr: [[Double]]){
        
//        var dataArr: [[Double]] = []

//        print(localArr[..<20])
        
        guard let inputArr = try? MLMultiArray(shape:[40,1,2], dataType:.double) else {
            print("ERROR in MLMultiArray")
            return
        }
        
        for i in 0..<40 {
            //                    inputArr[i] = dataArr[i] as [NSNumber]
            inputArr[[i as NSNumber,0,0]] = localArr[i][0] as NSNumber
            inputArr[[i as NSNumber,0,1]] = localArr[i][1] as NSNumber
        }
        let ainputArr = bmodelInput(input1: inputArr)
        //                let predOptions = MLPredictionOptions()
        //                predOptions.usesCPUOnly = true
        //                guard let out = try? alpha.prediction(input: ainputArr, options: predOptions) else {
        guard let out = try? alpha.prediction(input: ainputArr) else {
            
            print("ERROR: prediction failed")
            return
        }
//            print("predicting..")
//        let action = out.output1
        let featurePointer = UnsafePointer<Double>(OpaquePointer(out.output1.dataPointer))
//        func argmax(_ array: UnsafePointer<Double>, count: Int) -> (Int, Double) {
        var maxValue: Double = 0
        var maxIndex: vDSP_Length = 0
        vDSP_maxviD(featurePointer, 1, &maxValue, &maxIndex, vDSP_Length(4))
//        print(Int(maxIndex), maxValue)
        let actionVal = Int(maxIndex)
        action[actionVal] += 1
//        let (maxIndex, maxValue) = argmax(featurePointer, 43)
//        print(out.output1)
//        print(action)

    }
    
    func startAction(){
        
        if action[0] > 15 {
            print("none \(action)")
        } else if action[1] > 29 {
            print("left \(action)")
            NotificationCenter.default.post(name: Notification.Name("left"), object: nil)
            
        } else if action[2] > 5 {
            print("right \(action)")
            NotificationCenter.default.post(name: Notification.Name("right"), object: nil)
            
        } else if action[3] > 5{
            print("both \(action)")
//            NotificationCenter.default.post(name: Notification.Name("both"), object: nil)
            NotificationCenter.default.post(name: Notification.Name("right"), object: nil)

        }
        action = [0,0,0,0]
        countWindow = 0
    }
    
    func peripheral(_ peripheral: CBPeripheral, didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        switch characteristic.uuid {
        case HIDcharCBUUID:
            guard let characteristicData = characteristic.value else { return }
//            print(Int64(Date().timeIntervalSince1970 * 1000))
//            print("characteristic data", characteristicData)
            let byteArray = [UInt8](characteristicData)
            var tempArray: [UInt8] = []
//            print(byteArray)
            tempArray += byteArray
//            print(tempArray)
            let tempSplit = tempArray.split(separator: 10)
//            print(tempSplit)
            for idx in tempSplit[..<tempSplit.count] {
//                print(idx)
                if let tupStr = String(bytes: idx, encoding: .utf8){
//                    print(tupStr)
                    var tuple = tupStr.split(separator: ",")
                    if tuple.count == 2{
                        if let temp0 = Int(tuple[0]){
                            if let temp1 = Int(tuple[1]){
                                if (temp0 != 65535) && (temp1 != 65535){
                                    dataArr.append([Double(temp0)/1023, Double(temp1)/1023])
                                }
                                if dataArr.count > 40 {
                                    dataArr.removeFirst()
                                }
                            }

                        }
                    }
               
                }
            }
            tempArray = []
            if let temptemp = tempSplit.last {
                tempArray += temptemp
            }
            
//            if dataArr.count == 50 {
//                countWindow += 1
//                var sum0: Double = 0
//                var sum1: Double = 0
//                for i in dataArr {
//                    sum0 += i[0]
//                    sum1 += i[1]
//                }
////                let sum0 = (dataArr[..<49][0]).reduce(0, +)
////                let sum1 = (dataArr[..<49][1]).reduce(0, +)
////                print( Double(sum0) * (1.2 / 48))
////                print( Double(sum1) * (1.2 / 48))
////                print(dataArr[49][0])
////                print(dataArr[49][1])
//                var left = 0
//                var right = 0
////                print("-----------")
//                if ( Double(sum0) * (1.5 / 48) < Double (dataArr[49][0])){
////                    print("sensor 0 high")
//                    actionWindow[0] += 1
////                    NotificationCenter.default.post(name: Notification.Name("newBoop"), object: nil)
//                }
//                if ( Double(sum1) * (1.5 / 48) < Double (dataArr[49][1])){
////                    print("sensor 1 high")
//                    actionWindow[1] += 1
//                    //                    NotificationCenter.default.post(name: Notification.Name("newBoop"), object: nil)
//                }
////                print(left, right)
//                if countWindow >= 25 {
//                    if (actionWindow[0] + actionWindow[1]) < 20 {
//                        print("none")
//                    }
//                    else if actionWindow[1] > actionWindow[0] + 5 {
//                        NotificationCenter.default.post(name: Notification.Name("right"), object: nil)
//                        print("right")
//                    }
//                    else if actionWindow[0] > actionWindow[1] + 10 {
//                        NotificationCenter.default.post(name: Notification.Name("left"), object: nil)
//
//                        print("left")
//
//                    } else if (actionWindow[0] + actionWindow[1]) > 15 {
//                        NotificationCenter.default.post(name: Notification.Name("both"), object: nil)
//
//                        print("both")
//                    } else {
//                        print("none")
//                    }
//                    countWindow = 0
//                    actionWindow = [0,0]
//                }
//            }
            
////            print("end", Int64(Date().timeIntervalSince1970 * 1000))
            if dataArr.count == 40 {
//                print("in count")
                if flag == false {
                    threadRecurse()
                    flag = true
                }
            }
            
        default:
            print("Unhandled Characteristic UUID: \(characteristic.uuid)")
        }
    }
    

    
}
