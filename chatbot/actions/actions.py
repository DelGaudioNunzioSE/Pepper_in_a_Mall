# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher

import csv
from num2words import num2words #esiste word2num ma non me la fa installare


class ActionGroups(Action):

    def name(self) -> Text:
        return "action_groups_number"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        file_path = "./files/Gruppi.csv" #sono in chatbot

        try:
            # Apre il file in modalità lettura
            with open(file_path, mode='r', newline='', encoding='utf-8') as file:
                reader = csv.reader(file)
                # Conta il numero di righe (eccetto l'intestazione, se presente)
                num_lines = sum(1 for row in reader)

            # Restituisce il numero di righe come risposta
            dispatcher.utter_message(text=f"The groups are {num_lines-1}, with 3 or 4 people per group.") #-1 perchè il primo è l'header

        except FileNotFoundError:
            dispatcher.utter_message(text=f"Sorry, the file {file_path} was not found.")


        return []


class CompositionGroups(Action):

    def name(self) -> Text:
        return "action_groups_composition"
    
    def word_to_number(self, group_number):
        number_dict = {
            'one': 1,
            'two': 2,
            'three': 3,
            'four': 4,
            'five': 5,
            'six': 6,
            'seven': 7,
            'eight': 8,
            'nine': 9,
            'ten': 10,
            'eleven': 11,
            'twelve': 12,
            'thirteen': 13,
            'fourteen': 14,
            'fifteen': 15
        }

        group_number = group_number.strip().lower() #toglie spazi o maiuscole
        if group_number in number_dict:
            return number_dict[group_number]
        else:
            return group_number

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        group_number = tracker.get_slot('group_number')

        if not group_number:
            dispatcher.utter_message(text="Please provide the group number.")
            return []

        # Percorso del file CSV
        file_path = "./files/Gruppi.csv"

        try:
            with open(file_path, mode='r', newline='', encoding='utf-8') as file:
                reader = csv.reader(file)
                headers = next(reader)  # Legge l'intestazione del file CSV

                
                group_number = int(self.word_to_number(group_number))
                if group_number == None:
                    dispatcher.utter_message(text="The group number must be a valid number.")
                    return []

                # Cerca la riga corrispondente al numero del gruppo
                for i, row in enumerate(reader, start=2):  # Inizia da 2 per saltare l'intestazione
                    if i == group_number+1: #perchè salta l'intestazione

                        dispatcher.utter_message(text=f"The components of group {group_number} are: {row[1]} {row[2]}, {row[4]} {row[5]}, {row[7]} {row[8]}, {row[10]} {row[11]}.")
                        return []

                # Se il numero del gruppo non è trovato
                dispatcher.utter_message(text=f"Group {group_number} not found in the file.")
                return []

        except FileNotFoundError:
            dispatcher.utter_message(text=f"Sorry, the file {file_path} was not found.")
        
        return []



class ActionGroupsRanking(Action):

    def name(self) -> Text:
        return "action_groups_ranking"
    

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        file_path = "./files/Ranking.csv" #sono in chatbot

        
        with open(file_path, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.reader(file)
            header = next(reader)
            # Conta il numero di righe (eccetto l'intestazione, se presente)
            dispatcher.utter_message(text=f"The ranking is: ")
            for i, row in enumerate(reader):
                dispatcher.utter_message(text=f"Pos:{i+1}, Group:{row[0]}, Components:{row[1]} {row[2]}, {row[4]} {row[5]}, {row[7]} {row[8]}, {row[10]} {row[11]}, PFS:{row[13]} LFS:{row[14]} GFS:{row[15]} BFS:{row[16]} HFS:{row[17]} AFS: 0.{row[18]}")
                    
        return []
        

class ActionGroupRank(Action):

    def name(self) -> Text:
        return "action_group_rank"
    
    def word_to_number(self, group_number):
        number_dict = {
            'one': 1,
            'two': 2,
            'three': 3,
            'four': 4,
            'five': 5,
            'six': 6,
            'seven': 7,
            'eight': 8,
            'nine': 9,
            'ten': 10,
            'eleven': 11,
            'twelve': 12,
            'thirteen': 13,
            'fourteen': 14,
            'fifteen': 15
        }
        group_number = group_number.strip().lower() #toglie spazi o maiuscole
        if group_number in number_dict:
            return number_dict[group_number]
        else:
            return group_number

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        file_path = "./files/Ranking.csv" #sono in chatbot

        
        # Se specifica il group number con questo intento, vuole saperne la posizione o i punteggi
        group_number = tracker.get_slot('group_number')
        if group_number is not None:
            group_number = int(self.word_to_number(group_number))
            with open(file_path, mode='r', newline='', encoding='utf-8') as file:
                reader = csv.reader(file)
                header = next(reader)
                for i, row in enumerate(reader):
                    if int(row[0]) == group_number:
                        dispatcher.utter_message(text=f"The group:{row[0]} ended in pos:{i+1}. Summary: components:{row[1]} {row[2]}, {row[4]} {row[5]}, {row[7]} {row[8]}, {row[10]} {row[11]}, PFS:{row[13]} LFS:{row[14]} GFS:{row[15]} BFS:{row[16]} HFS:{row[17]} AFS: 0.{row[18]}")
                        return []
                dispatcher.utter_message(text=f"The group {group_number} wasn't in the contest!")
        return[]
