//
//  LeaderboardViewController.swift
//  Gravity Rush
//
//  Created by Ryan Armiger on 08/02/2019.
//  Copyright © 2019 Ryan Armiger. All rights reserved.
//

import UIKit
import FirebaseDatabase

class LeaderboardViewController: UIViewController, UITableViewDelegate, UITableViewDataSource {
    

    @IBOutlet weak var searchBar: UISearchBar!
    @IBOutlet private weak var leaderTable: UITableView!
    @IBOutlet weak var buttonText: UIButton!
    @IBOutlet weak var navView: UIView!
    var ref: DatabaseReference?
    let thisUser = "myUser"
    var locationState = String()
    var leaderNameData: [String] = []
    var leaderScoreData: [String] = []
    var leaderScoreSortedData: [String] = []
    var leaderNameSortedData: [String] = []
    var searchFriendData: [String] = []
    var savedFriends: [String] = []
    var friendIndeces: [Int] = []
    var positionData: [Int] = []
    var myLocation = Int()
    var topConstraint = NSLayoutConstraint()
    var bottomConstraint = NSLayoutConstraint()
    var steve = 2
    var searching = false
    
    @IBAction func addFriendButton(_ sender: UIButton) {
        if steve == 3{
            steve = 2
            searchBar.resignFirstResponder()
            buttonText.setTitle("+ Add Friends", for: .normal)
            searchBar.isHidden = true
            NSLayoutConstraint.deactivate([bottomConstraint])
            NSLayoutConstraint.deactivate([topConstraint])
            self.topConstraint = leaderTable.topAnchor.constraint(equalTo: navView.bottomAnchor)
            NSLayoutConstraint.activate([topConstraint])
        }
        else{
            steve = 3
            buttonText.setTitle("Back", for: .normal)
            searchBar.isHidden = false
            NSLayoutConstraint.deactivate([topConstraint])
            self.bottomConstraint = searchBar.bottomAnchor.constraint(equalTo: leaderTable.topAnchor)
            self.topConstraint = searchBar.topAnchor.constraint(equalTo: navView.bottomAnchor)
            NSLayoutConstraint.activate([topConstraint])
            NSLayoutConstraint.activate([bottomConstraint])
        }
        self.leaderTable.reloadData()
    }

    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        if steve == 2{
            return(leaderNameSortedData.count)
        }
        else{
            if searching{
                return(searchFriendData.count)
            }
            else{
                return(leaderNameData.count)
            }
        }
    }
    
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        if steve == 2{
            let cell = tableView.dequeueReusableCell(withIdentifier: "leaderCell", for: indexPath) as! ViewControllerTableViewCell
            cell.leaderNameLabel.text = leaderNameSortedData[indexPath.row]
            cell.leaderScoreLabel.text = leaderScoreSortedData[indexPath.row]
            if positionData[indexPath.row] == -1 {
                cell.position.text = "..."
            }
            else {
                cell.position.text = String(positionData[indexPath.row])
            }
            if leaderNameSortedData[indexPath.row] == thisUser{
                cell.backgroundColor = #colorLiteral(red: 0.6000000238, green: 0.6000000238, blue: 0.6000000238, alpha: 1)
            }
            else if savedFriends.contains(leaderNameSortedData[indexPath.row]){
                cell.backgroundColor = #colorLiteral(red: 0.2549019754, green: 0.2745098174, blue: 0.3019607961, alpha: 1)
            }
            else{
                cell.backgroundColor = #colorLiteral(red: 0, green: 0, blue: 0, alpha: 1)
            }
            return(cell)
        }
        else{
            let cell = tableView.dequeueReusableCell(withIdentifier: "addFriendCell", for: indexPath) as! ViewControllerSearchFriendCell
            if searching{
                cell.friendName.text = searchFriendData[indexPath.row]
                if savedFriends.contains(searchFriendData[indexPath.row]) {
                    cell.backgroundColor = #colorLiteral(red: 0.9764705896, green: 0.6806376668, blue: 0.9618322281, alpha: 1)
                }
                else{
                    cell.backgroundColor = #colorLiteral(red: 0, green: 0, blue: 0, alpha: 1)
                }
            }
            else{
                cell.friendName.text = leaderNameData[indexPath.row]
                if savedFriends.contains(leaderNameData[indexPath.row]){
                    cell.backgroundColor = #colorLiteral(red: 0.9764705896, green: 0.6806376668, blue: 0.9618322281, alpha: 1)
                }
                else{
                    cell.backgroundColor = #colorLiteral(red: 0, green: 0, blue: 0, alpha: 1)
                }
            }
            return(cell)
        }
    }
    
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        if steve == 3{
            let myIndex = indexPath.row
            if UserDefaults.standard.object(forKey: "savedFriends") != nil{
                savedFriends = UserDefaults.standard.object(forKey: "savedFriends") as! [String]
            }
            if searching{
                if savedFriends.contains(searchFriendData[myIndex]){
                    savedFriends = savedFriends.filter { $0 !=  searchFriendData[myIndex]}
                    UserDefaults.standard.set(savedFriends, forKey: "savedFriends")
                }
                else{
                    savedFriends.append(searchFriendData[myIndex])
                    UserDefaults.standard.set(savedFriends, forKey: "savedFriends")
                }
                self.newMethod()
                searchBar.isHidden = false
            }
            else{
                if savedFriends.contains(leaderNameData[myIndex]){
                    savedFriends = savedFriends.filter { $0 !=  leaderNameData[myIndex]}
                    UserDefaults.standard.set(savedFriends, forKey: "savedFriends")
                }
                else{
                    savedFriends.append(leaderNameData[myIndex])
                    UserDefaults.standard.set(savedFriends, forKey: "savedFriends")
                }
                self.newMethod()
                searchBar.isHidden = false
            }
        }
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        self.newMethod()
        NotificationCenter.default.addObserver(self, selector: #selector(reloadFunc), name: Notification.Name("reloadLeader"), object: nil)

    }
    
    @objc func reloadFunc() {
        print("actually working")
        self.newMethod()
    }
    
    func newMethod() {
        
        if UserDefaults.standard.object(forKey: "oldScore") != nil{
            print("yoop")
        }
        else {
            UserDefaults.standard.set(0, forKey: "oldScore")
        }
        
        print("yoot")
        if UserDefaults.standard.object(forKey: "savedFriends") != nil{
            savedFriends = UserDefaults.standard.object(forKey: "savedFriends") as? [String] ?? [""]
            print("saved", savedFriends)
        }
        else {
            print("yoot")
        }
        
        searchBar.isHidden = true
        
        if steve == 2{
            NSLayoutConstraint.deactivate([topConstraint])
            NSLayoutConstraint.deactivate([bottomConstraint])
            self.topConstraint = leaderTable.topAnchor.constraint(equalTo: navView.bottomAnchor)
            NSLayoutConstraint.activate([topConstraint])
        }
        else{
            NSLayoutConstraint.deactivate([topConstraint])
            self.bottomConstraint = searchBar.bottomAnchor.constraint(equalTo: leaderTable.topAnchor)
            self.topConstraint = searchBar.topAnchor.constraint(equalTo: navView.bottomAnchor)
            NSLayoutConstraint.activate([topConstraint])
            NSLayoutConstraint.activate([bottomConstraint])
        }
        
        ref = Database.database().reference()
        var prevData = String()
        var prevDataIndex = 1
        guard let db = ref else {
            print("db nil")
            return
        }
        
        let old = UserDefaults.standard.object(forKey: "oldScore") as? Int
        
        db.child("gameUsers/id18").updateChildValues(["HighScore": old ?? 0])
        
        let test = db.child("gameUsers").queryOrdered(byChild: "HighScore")
        test.observe(DataEventType.value, with:{(snapshot: DataSnapshot) in
                self.leaderNameData.removeAll()
                self.leaderScoreData.removeAll()
                self.leaderNameSortedData.removeAll()
                self.leaderScoreSortedData.removeAll()
                self.positionData.removeAll()
                
                for gameUsers in snapshot.children.allObjects as![DataSnapshot]{
                    let userObject = gameUsers.value as? [String: AnyObject]
                    
                    let userName = userObject?["username"]
                    let userScore = userObject?["HighScore"]

                    self.leaderNameData.append(userName as? String ?? "ERROR: Append User")
                    self.leaderScoreData.append(String(userScore as? Int ?? 0) )
                }
            self.leaderNameData.reverse()
            self.leaderScoreData.reverse()
            
            let end = self.leaderNameData.count-1
            print(end)
            
            for i in 0...end{
                if i < 5{
                    if self.leaderNameData[i]==self.thisUser{
                        //self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                        self.locationState = "top5"
                    }
                }
                else if i > 4 && i < 10{
                    if self.leaderNameData[i]==self.thisUser {
                        //self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                        self.locationState = "top10"
                    }
                }
                else if i > 9 && i < (end-5){
                    if self.leaderNameData[i]==self.thisUser{
                        //self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                        self.locationState = "notTop"
                        self.myLocation = i
                    }
                }
                else if self.leaderNameData[i]==self.thisUser{
                    //self.oldScore = Int(self.leaderScoreData[i]) ?? 0
                    self.locationState = "bottom5"
                    self.myLocation = i
                }
                
                if self.savedFriends.contains(self.leaderNameData[i]){
                    self.friendIndeces.append(i)
                }
            }
            
            if self.locationState == "top5"||self.locationState == "top10"{
                for i in 0...14{
                    self.leaderScoreSortedData.append(self.leaderScoreData[i])
                    self.leaderNameSortedData.append(self.leaderNameData[i])
                    if self.leaderScoreData[i] == prevData{
                        self.positionData.append(prevDataIndex)
                    }
                    else{
                        self.positionData.append(i+1)
                        prevDataIndex = i+1
                    }
                    prevData = self.leaderScoreData[i]
                }
                
                for i in 15...end{
                    if prevDataIndex != i && prevDataIndex != i+1{
                        if self.leaderNameSortedData.last != "..."{
                            self.leaderNameSortedData.append("...")
                            self.positionData.append(-1)
                            self.leaderScoreSortedData.append("...")
                        }
                    }
                    if self.friendIndeces.contains(i){
                        self.leaderScoreSortedData.append(self.leaderScoreData[i])
                        self.leaderNameSortedData.append(self.leaderNameData[i])
                        if self.leaderScoreData[i] == prevData{
                            self.positionData.append(prevDataIndex)
                        }
                        else{
                            self.positionData.append(i+1)
                            prevDataIndex = i+1
                        }
                        prevData = self.leaderScoreData[i]
                    }
                }
            }
            else if self.locationState == "notTop"{
                //make it so it doesn't work if it's out of range (i.e. if bottom 5 or there are less than 5 players
                for i in 0...4{
                    self.leaderScoreSortedData.append(self.leaderScoreData[i])
                    self.leaderNameSortedData.append(self.leaderNameData[i])
                    if self.leaderScoreData[i] == prevData{
                        self.positionData.append(prevDataIndex)
                    }
                    else{
                        self.positionData.append(i+1)
                        prevDataIndex = i+1
                    }
                    prevData = self.leaderScoreData[i]

                }
                
                for i in 5...(self.myLocation-6){
                    if prevDataIndex != i && prevDataIndex != i-1{
                        if self.leaderNameSortedData.last != "..."{
                            self.leaderNameSortedData.append("...")
                            self.positionData.append(-1)
                            self.leaderScoreSortedData.append("...")
                        }
                    }
                    if self.friendIndeces.contains(i){
                        self.leaderScoreSortedData.append(self.leaderScoreData[i])
                        self.leaderNameSortedData.append(self.leaderNameData[i])
                        if self.leaderScoreData[i] == prevData{
                            self.positionData.append(prevDataIndex)
                        }
                        else{
                            self.positionData.append(i+1)
                            prevDataIndex = i+1
                        }
                        prevData = self.leaderScoreData[i]
                    }
                }
                
                for i in (self.myLocation-5)...(self.myLocation+5){
                    self.leaderScoreSortedData.append(self.leaderScoreData[i])
                    self.leaderNameSortedData.append(self.leaderNameData[i])
                    if self.leaderScoreData[i] == prevData{
                        self.positionData.append(prevDataIndex)
                    }
                    else{
                        self.positionData.append(i+1)
                        prevDataIndex = i+1
                    }
                    prevData = self.leaderScoreData[i]

                }
                
                for i in (self.myLocation+6)...end{
                    if prevDataIndex != i && prevDataIndex != i-1{
                        if self.leaderNameSortedData.last != "..."{
                            self.leaderNameSortedData.append("...")
                            self.positionData.append(-1)
                            self.leaderScoreSortedData.append("...")
                        }
                    }
                    if self.friendIndeces.contains(i){
                        self.leaderScoreSortedData.append(self.leaderScoreData[i])
                        self.leaderNameSortedData.append(self.leaderNameData[i])
                        if self.leaderScoreData[i] == prevData{
                            self.positionData.append(prevDataIndex)
                        }
                        else{
                            self.positionData.append(i+1)
                            prevDataIndex = i+1
                        }
                        prevData = self.leaderScoreData[i]
                    }
                }
            }
            else if self.locationState == "bottom5"{
                
                for i in 0...4{
                    self.leaderScoreSortedData.append(self.leaderScoreData[i])
                    self.leaderNameSortedData.append(self.leaderNameData[i])
                    if self.leaderScoreData[i] == prevData{
                        self.positionData.append(prevDataIndex)
                    }
                    else{
                        self.positionData.append(i+1)
                        prevDataIndex = i+1
                    }
                    prevData = self.leaderScoreData[i]
                }
                
                for i in 5...(self.myLocation-6){
                    if (prevDataIndex != i) && (prevDataIndex != i-1){
                        if self.leaderNameSortedData.last != "..."{
                            self.leaderNameSortedData.append("...")
                            self.positionData.append(-1)
                            self.leaderScoreSortedData.append("...")
                        }
                    }
                    if self.friendIndeces.contains(i){
                        print("i = ", i)
                        print("prev = ", prevDataIndex)
                        self.leaderScoreSortedData.append(self.leaderScoreData[i])
                        self.leaderNameSortedData.append(self.leaderNameData[i])
                        if self.leaderScoreData[i] == prevData{
                            self.positionData.append(prevDataIndex)
                        }
                        else{
                            self.positionData.append(i+1)
                            prevDataIndex = i+1
                        }
                        prevData = self.leaderScoreData[i]
                    }
                }
                
                for i in (self.myLocation-5)...(self.myLocation-1){
                    self.leaderScoreSortedData.append(self.leaderScoreData[i])
                    self.leaderNameSortedData.append(self.leaderNameData[i])
                    if self.leaderScoreData[i] == prevData{
                        self.positionData.append(prevDataIndex)
                    }
                    else{
                        self.positionData.append(i+1)
                        prevDataIndex = i+1
                    }
                    prevData = self.leaderScoreData[i]
                }
                
                for i in (self.myLocation)...end{
                    self.leaderScoreSortedData.append(self.leaderScoreData[i])
                    self.leaderNameSortedData.append(self.leaderNameData[i])
                    if self.leaderScoreData[i] == prevData{
                        self.positionData.append(prevDataIndex)
                    }
                    else{
                        self.positionData.append(i+1)
                        prevDataIndex = i+1
                    }
                    prevData = self.leaderScoreData[i]
                }
                
            }
            if self.leaderNameSortedData.last == "..."{
                self.leaderNameSortedData.removeLast()
                self.positionData.removeLast()
                self.leaderScoreSortedData.removeLast()
            }
            self.leaderTable.reloadData()
        })

        //Lets you push data to the database
       //let ref = Database.database().reference()
       //ref.childByAutoId().setValue(["username":"Becky", "HighScore":"2"])
        //update
        
    }
    
   
    
    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
    }
    */

}

extension LeaderboardViewController: UISearchBarDelegate {
    func searchBar( _ searchBar: UISearchBar, textDidChange searchText: String){
        searchFriendData = leaderNameData.filter({$0.prefix(searchText.count) == searchText})
        searching = true
        self.leaderTable.reloadData()
    }
    
    func searchBarCancelButtonClicked(_ searchBar: UISearchBar) {
        searching = false
        searchBar.text = ""
        searchBar.resignFirstResponder()
        self.leaderTable.reloadData()
    }
}
