# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet

import csv
import json


class ActionGroups(Action):

    def name(self) -> Text:
        return "action_groups_number"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        file_path = "./files/Gruppi.json" #sono in chatbot

        try:
            # Apre il file in modalità lettura
            with open(file_path, mode='r', encoding='utf-8') as file:
                data = json.load(file)

            # Restituisce il numero di righe come risposta
            num_lines = len(data["Group ID"])
            dispatcher.utter_message(text=f"The groups are {num_lines}, with 3 or 4 people per group. Tell me if you want know the global ranking or who is the winner.") #-1 perchè il primo è l'header

        except FileNotFoundError:
            dispatcher.utter_message(text=f"Sorry, the file {file_path} was not found.")


        return []


class CompositionGroups(Action):

    def name(self) -> Text:
        return "action_groups_composition"
    
    def word_to_number(self, group_number):
        if isinstance(group_number, int): #se è un intero non faccio il controllo
            return group_number #lo restituisco direttamente in output
        
        number_dict = {
            'one': 1,
            'winner': 1,
            'first':1,
            '1': 1,

            'two': 2,
            'second':2,
            '2':2,

            'three': 3,
            'third':3,
            '3':3,

            'four': 4,
            'fourth':4,
            '4':4,

            'five': 5,
            'fifth': 5,
            '5':5,

            'six': 6,
            'sixth':6,
            '6':6,

            'seven': 7,
            'seventh': 7,
            '7':7,

            'eight': 8,
            'eighth': 8,
            '8':8,

            'nine': 9,
            'nineth':9,
            '9':9,

            'ten': 10,
            'tenth':10,
            '10':10,

            'eleven': 11,
            'eleventh': 11,
            '11':11,

            'twelve': 12,
            'twelveth': 12,
            '12':12,

            'thirteen': 13,
            'thirteenth': 13,
            '13':13,

            'fourteen': 14,
            'fourteenth':14,
            '14':14,

            'last': 15,
            'fifteen': 15,
            'fifteenth': 15,
            '15':15
        }

        group_number = group_number.strip().lower() #toglie spazi o maiuscole
        if group_number in number_dict:
            return number_dict[group_number]
        else:
            return -1

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        group_number = tracker.get_slot('group_number')
        ranking_or = next(tracker.get_latest_entity_values("rankings"), None)
        group_member_or = next(tracker.get_latest_entity_values("group_member"), None)

        if not group_number and not ranking_or and not group_member_or:
            dispatcher.utter_message(text="Do you want know something about the competition? Please provide the group number.")
            return []

        # Percorso del file CSV
        file_path = "./files/Gruppi.json"

        try:
            with open(file_path, mode='r', encoding='utf-8') as file:
                data = json.load(file)  # Legge l'intestazione del file CSV

                #singolo componente del gruppo
                if group_member_or is not None:
                    group_member = str(group_member_or)
                    for group, members in data["group_members"].items():
                        if any(group_member.upper() in str(member).upper() for member in members):           
                            grp = data["Group ID"][group]
                            dispatcher.utter_message(text = f"{group_member} is in group {grp}, the components of this group are: " + ", ".join(members[:4]) + ".")
                            return [SlotSet("group_number", group_number)]
                    dispatcher.utter_message(text = f"Sorry, {group_member_or} is not a competitor")
                    return []

                #Quali sono i componenti del gruppo ad un determinato rank
                if ranking_or is not None:
                    ranking = self.word_to_number(ranking_or)
                    for group, rank in data["ranking_position"].items():
                        grp = data["Group ID"][group]
                        if int(ranking) == rank:
                            members = data["group_members"][group] 
                            dispatcher.utter_message(text = f"The components of group {grp}, which classified {str(ranking_or)} are: " + ", ".join(members[:4]) + ".")
                            return [SlotSet("group_number", grp)]                
                    dispatcher.utter_message(text = f"The global ranking doesn't have this position. Ask me another position, i'll try to answer!")
                    return []

                try:
                    group_number = int(group_number)
                except:
                    dispatcher.utter_message(text="Sorry, i didn't understand. Can you repeat please?") #specificare corretta posizione
                    return[]
                if group_number == None:
                    dispatcher.utter_message(text="Sorry, i didn't understand. Can you repeat please?") #specificare corretta posizione
                    return []
                
                
                # Cerca la riga corrispondente al numero del gruppo
                dispatcher.utter_message(text=f"Let me check...")
                for group, members in data["group_members"].items():
                    grp = data["Group ID"][group]
                    if grp == group_number: 
                        dispatcher.utter_message(text = f"The components of group {group_number} are: " + ", ".join(members[:4]) + ". Ask me something about this group")
                        return []

                # Se il numero del gruppo non è trovato
                dispatcher.utter_message(text=f"Group {group_number} not found in the database of competition.")
                return []

        except FileNotFoundError:
            dispatcher.utter_message(text=f"Sorry, the file {file_path} was not found in my database.")
        
        return []



class ActionGroupsRanking(Action):

    def name(self) -> Text:
        return "action_groups_ranking"
    

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        file_path = "./files/Gruppi.json" #sono in chatbot

        #Classifica globale ma prima la ordino per rank
        with open(file_path, mode='r', encoding='utf-8') as file:
            data = json.load(file)          
            ranking_data = sorted(
                [(position, data["Group ID"][group], data["AFS"][group]) for group, position in data["ranking_position"].items()],
                key=lambda x: x[0] 
            )            
            dispatcher.utter_message(text="The ranking is: ")
            for rank, (position, grp, afs) in enumerate(ranking_data, start=1):
                dispatcher.utter_message(text=f"Position: {rank}, Group: {grp}, AFS: {afs}")
                    
        return []
        

class ActionGroupRank(Action):

    def name(self) -> Text:
        return "action_group_rank"
    
    def word_to_number(self, group_number):

        if isinstance(group_number, int): #se è un intero non faccio il controllo
            return group_number #lo restituisco direttamente in output

        number_dict = {
            'one': 1,
            'winner': 1,
            'first':1,
            '1': 1,

            'two': 2,
            'second':2,
            '2':2,

            'three': 3,
            'third':3,
            '3':3,

            'four': 4,
            'fourth':4,
            '4':4,

            'five': 5,
            'fifth': 5,
            '5':5,

            'six': 6,
            'sixth':6,
            '6':6,

            'seven': 7,
            'seventh': 7,
            '7':7,

            'eight': 8,
            'eighth': 8,
            '8':8,

            'nine': 9,
            'nineth':9,
            '9':9,

            'ten': 10,
            'tenth':10,
            '10':10,

            'eleven': 11,
            'eleventh': 11,
            '11':11,

            'twelve': 12,
            'twelveth': 12,
            '12':12,

            'thirteen': 13,
            'thirteenth': 13,
            '13':13,

            'fourteen': 14,
            'fourteenth':14,
            '14':14,

            'last': 15,
            'fifteen': 15,
            'fifteenth': 15,
            '15':15
        }
        group_number = group_number.strip().lower() #toglie spazi o maiuscole
        if group_number in number_dict:
            return number_dict[group_number]
        else:
            return -1
        
        
    def summary_group(self, group, rank, members, afs):
        text=f"The group {int(group)} ended in position {rank}. This is a summary for you. The components of group {int(group)} are: " + ", ".join(members[:4]) + "." +f" the AFS is: {afs}"
        return text
    
    def detect_score(self, afs, score):
        if score == "AFS":
            sc = afs
        else:
            return -1
        return sc
        

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        file_path = "./files/Gruppi.json" #sono in chatbot

        

        
        # Se specifica il group number con questo intento, vuole saperne la posizione o i punteggi
        group_number = tracker.get_slot('group_number')
        score = next(tracker.get_latest_entity_values("score"), None) #prendo l'entità per non avere lo slot sennò lo memorizza tutta la conversazione
        ranking = next(tracker.get_latest_entity_values("rankings"), None)

        # Who is the winner?
        if ranking is not None:
            ranking = int(self.word_to_number(ranking))
            if ranking == -1:
                dispatcher.utter_message(text=f"Sorry, can you repeat? I didn't understand the position..")
                return []

            with open(file_path, mode='r', encoding='utf-8') as file:
                data = json.load(file)
                for group, rank in data["ranking_position"].items():
                    grp = data["Group ID"][group]
                    if int(rank) == ranking:
                        dispatcher.utter_message(self.summary_group(grp,rank,data["group_members"][group], data["AFS"][group]))
                        return [SlotSet("group_number", grp)] #cambia il numero del gruppo
                    
        # Posizione di un gruppo generico
        if group_number is not None and score == None:
            try:
                group_number = int(group_number)
            except:
                dispatcher.utter_message(text="Sorry, i didn't understand. Can you repeat please?") #specificare corretta posizione
                return[]
            if group_number == None:
                dispatcher.utter_message(text=f"Sorry, can you repeat? I didn't understand the group number..")
                return []

            with open(file_path, mode='r', encoding='utf-8') as file:
                data = json.load(file)
                for group, rank in data["ranking_position"].items():
                    grp = data["Group ID"][group]
                    if grp == group_number:
                        dispatcher.utter_message(self.summary_group(grp,rank,data['group_members'][group], data['AFS'][group]))
                        return []
                dispatcher.utter_message(text=f"The group {group_number} wasn't in the contest")
                return []

        # Punteggi di un gruppo generico
        elif group_number is not None and score is not None:
            try:
                group_number = int(group_number)
            except:
                dispatcher.utter_message(text="Sorry, i didn't understand. Can you repeat please?") #specificare corretta posizione
                return[]
            
            if group_number == None:
                dispatcher.utter_message(text=f"Sorry, can you repeat? I didn't understand the group number..")
                return []

            with open(file_path, mode='r', encoding='utf-8') as file:
                data = json.load(file)
                for group, rank in data["ranking_position"].items():
                     grp = data["Group ID"][group]
                     if grp == group_number:
                        score = str(score).upper()
                        #capisco di che metrica vuole il risultato
                        sc = self.detect_score(data["AFS"][group], score)
                        #se non è scritta in questo modo gli dò il risultato generale
                        if sc == -1:
                            dispatcher.utter_message(f"I'm not sure if I understood, but let me check... {self.summary_group(grp,rank,data['group_members'][group], data['AFS'][group])}")
                            return []
                        if sc > 0.8:
                            txt = "An impressive result!"
                        else:
                            txt = "A good result."
                        dispatcher.utter_message(text=f"The group:{group_number} {score} is {sc}. " + txt)
                        return []
                dispatcher.utter_message(text=f"The group {group_number} wasn't in the competition")
                return[]
                
        dispatcher.utter_message(text=f"Sorry, I was not payng attention, can you repeat?")  
        return[]