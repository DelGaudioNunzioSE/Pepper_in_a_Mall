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
        if isinstance(group_number, int): #se è un intero non faccio il controllo
            return group_number #lo restituisco direttamente in output
        
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
            return -1

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

                
                group_number = int(group_number)
                if group_number == None:
                    dispatcher.utter_message(text="Sorry, i didn't understand. Can you repeat please?") #specificare corretta posizione
                    return []

                # Cerca la riga corrispondente al numero del gruppo
                dispatcher.utter_message(text=f"Let me check...")
                for i, row in enumerate(reader, start=2):  # Inizia da 2 per saltare l'intestazione
                    if i == group_number+1: #perchè salta l'intestazione

                        dispatcher.utter_message(text=f"The components of group {group_number} are: {row[1]} {row[2]}, {row[4]} {row[5]}, {row[7]} {row[8]}, {row[10]} {row[11]}.")
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
        
        file_path = "./files/Ranking.csv" #sono in chatbot

        
        with open(file_path, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.reader(file)
            header = next(reader)
            # Conta il numero di righe (eccetto l'intestazione, se presente)
            dispatcher.utter_message(text=f"The ranking is: ")
            for i, row in enumerate(reader):
                dispatcher.utter_message(text=f"Position:{i+1}, Group:{row[0]}")
                    
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
        
    def summary_group(self, row, i):
        text=f"The group {row[0]} ended in position {i+1}. This is a summary for you. The components are:{row[1]} {row[2]}, {row[4]} {row[5]}, {row[7]} {row[8]}, {row[10]} {row[11]}, PFS:{row[13]} LFS:{row[14]} GFS:{row[15]} BFS:{row[16]} HFS:{row[17]} AFS: 0.{row[18]}"
        return text
    
    def detect_score(self, row, score):
        if score == "PFS":
            sc = row[13]
        elif score == "LFS":
            sc = row[14]
        elif score == "GFS":
            sc = row[15]
        elif score == "BFS":
            sc = row[16]
        elif score == "HFS":
            sc = row[17]
        elif score == "AFS":
            sc = row[18]
        else:
            return -1
        return sc
        

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        
        file_path = "./files/Ranking.csv" #sono in chatbot

        

        
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

            with open(file_path, mode='r', newline='', encoding='utf-8') as file:
                reader = csv.reader(file)
                header = next(reader)
                for i, row in enumerate(reader):
                    if int(i+1) == ranking:
                        dispatcher.utter_message(self.summary_group(row,i))
                        return [SlotSet("group_number", int(row[0]))] #cambia il numero del gruppo
                    
        # Posizione di un gruppo generico
        if group_number is not None and score == None:
            group_number = int(group_number)
            if group_number == None:
                dispatcher.utter_message(text=f"Sorry, can you repeat? I didn't understand the group number..")
                return []

            with open(file_path, mode='r', newline='', encoding='utf-8') as file:
                reader = csv.reader(file)
                header = next(reader)
                for i, row in enumerate(reader):
                    if int(row[0]) == group_number:
                        dispatcher.utter_message(self.summary_group(row,i))
                        return []
                dispatcher.utter_message(text=f"The group {group_number} wasn't in the contest")
                return []

        # Punteggi di un gruppo generico
        elif group_number is not None and score is not None:
            group_number = int(group_number)
            if group_number == None:
                dispatcher.utter_message(text=f"Sorry, can you repeat? I didn't understand the group number..")
                return []

            with open(file_path, mode='r', newline='', encoding='utf-8') as file:
                reader = csv.reader(file)
                header = next(reader)
                for i, row in enumerate(reader):
                    if int(row[0]) == group_number:
                        score = str(score).upper()
                        #capisco di che metrica vuole il risultato
                        sc = self.detect_score(row,score)
                        #se non è scritta in questo modo gli dò il risultato generale
                        if sc == -1:
                            dispatcher.utter_message(f"I'm not sure if I understood, but let me check... {self.summary_group(row,i)}")
                            return []
                        dispatcher.utter_message(text=f"The group:{row[0]} {score} is {sc}")
                        return []
                dispatcher.utter_message(text=f"The group {group_number} wasn't in the competition")
                return[]
                
        dispatcher.utter_message(text=f"Sorry, I was not payng attention, can you repeat?")  
        return[]
