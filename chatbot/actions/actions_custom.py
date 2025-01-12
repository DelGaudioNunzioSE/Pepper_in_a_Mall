# This files contains your custom actions which can be used to run
# custom Python code.
#
# See this guide on how to implement these action:
# https://rasa.com/docs/rasa/custom-actions


# This is a simple example for a custom action which utters "Hello World!"

import json
from typing import Any, Text, Dict, List

from rasa_sdk import Action, Tracker
from rasa_sdk.executor import CollectingDispatcher
from rasa_sdk.events import SlotSet

import csv


traj_dict={
    1:"PINCO PALLINO",
    2:"UNISA",
    3:"COGNITIVE STORE",
    4:"PEPPER'S CAFE",
    5:"AV CAMERAS",
    6:"DATA ANALYSIS INTIMO",
    7:"NLP FOR KIDS"

}
class ActionCount(Action):
    def name(self) -> Text:
        return "action_count"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        with open('./files/results.txt', 'r') as file:
            data = json.load(file)
        people = data['people']

        # Recupera tutte le entità riconosciute
        attributes = tracker.latest_message['entities']
        
        # Filtro iniziale che include tutte le persone
        filtered_people = people

        # Applica i filtri in base alle entità
        for attribute in attributes:
            if attribute['entity'] == "attribute":
                value = attribute['value']
                
                if value == "hat":
                    filtered_people = [p for p in filtered_people if p.get('hat') == 'Yes']
                elif value == "bag":
                    filtered_people = [p for p in filtered_people if p.get('bag') == 'Yes']
                elif value == "male":
                    filtered_people = [p for p in filtered_people if p.get('gender') == 'Male']
                elif value == "female":
                    filtered_people = [p for p in filtered_people if p.get('gender') == 'Female']
        
        # Conta il numero di persone filtrate
        count = len(filtered_people)
        if count > 0:
            dispatcher.utter_message(text=f"There are {count} people matching your criteria.")
        else:
            dispatcher.utter_message(text="No people match your criteria.")
        
        return []



class ActionGetPersonInfo(Action):
    def name(self) -> Text:
        return "action_get_person_info"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        with open('./files/results.txt', 'r') as file:
            data = json.load(file)
        people = data['people']
        # Recupera l'ID dalla frase dell'utente
        person_id = next(tracker.get_latest_entity_values('person'), None)
        if person_id:
            person_id = int(person_id)
            person = next((p for p in people if p['id'] == person_id), None)

            if person:
                dispatcher.utter_message(text=f"Information about the person with ID {person_id}: {person}.")
            else:
                dispatcher.utter_message(text=f"No person found with ID {person_id}.")
        else:
            dispatcher.utter_message(text="I didn't understand the ID. Can you please repeat it?")

        return []

class ActionAttribute(Action):
    def name(self) -> Text:
        return "action_attribute"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:
        with open('./files/results.txt', 'r') as file:
            data = json.load(file)
        people = data['people']

        # Recupera tutte le entità riconosciute
        attributes = tracker.latest_message['entities']
        
        # Filtro iniziale che include tutte le persone
        filtered_people = people

        # Applica i filtri in base alle entità
        for attribute in attributes:
            if attribute['entity'] == "attribute":
                value = attribute['value']
                
                if value == "hat":
                    filtered_people = [p for p in filtered_people if p.get('hat') == 'Yes']
                elif value == "bag":
                    filtered_people = [p for p in filtered_people if p.get('bag') == 'Yes']
                elif value == "male":
                    filtered_people = [p for p in filtered_people if p.get('gender') == 'Male']
                elif value == "female":
                    filtered_people = [p for p in filtered_people if p.get('gender') == 'Female']
        
        # Conta il numero di persone filtrate
        count = len(filtered_people)
        if count > 0:
            id_founded=[p.get('id') for p in filtered_people]
            dispatcher.utter_message(text=f"There are {count} people matching your criteria. And they are : {id_founded}, if you want information by one, specify the ID")
        else:
            dispatcher.utter_message(text="Sorry, there's no people match your criteria.")
        
        return [SlotSet("person", id_founded)]

class ActionGetTrajectories(Action):
    def name(self) -> Text:
        return "action_get_trajectories"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        with open('./files/results.txt', 'r') as file:
            data = json.load(file)
        people = data['people']

        # Recupera attributi dal messaggio dell'utente
        attributes = tracker.latest_message['entities']
        if(len(attributes)>0):
            attributes = [attribute for attribute in attributes if attribute['entity'] == "attribute"]
        # Filtra le persone in base agli attributi
        filtered_people=[]
        if(len(attributes)>0):
            filtered_people = people
            for attribute in attributes:
                if attribute['entity'] == "attribute":
                    value = attribute['value']
                    if value == "hat":
                        filtered_people = [p for p in filtered_people if p.get('hat') == 'Yes']
                    elif value == "bag":
                        filtered_people = [p for p in filtered_people if p.get('bag') == 'Yes']
                    elif value == "male":
                        filtered_people = [p for p in filtered_people if p.get('gender') == 'Male']
                    elif value == "female":
                        filtered_people = [p for p in filtered_people if p.get('gender') == 'Female']

            # Verifica se è stata trovata una persona corrispondente
            if len(filtered_people) > 0:
                last_seen_info = filtered_people[0].get('trajectory', [])

                
                if len(last_seen_info) > 0:
                    # Calcola il nuovo indice
                    if len(last_seen_info)>0:  
                        location = last_seen_info[-1]
                        location = traj_dict[location]
                        dispatcher.utter_message(
                            text=f"Let me check in my database. {filtered_people[0].get('id')} should have passed close to store {location}."
                        )
                    else:
                        dispatcher.utter_message(
                            text=f"Let me check in my database. I don't have location, i'm sorry."
                        )

                    # Aggiorna lo slot con il nuovo indice
                    return [SlotSet("person", filtered_people[0].get('id'))]
                else:
                    dispatcher.utter_message(
                        text=f"I found {filtered_people[0].get('id')} but I don’t have trajectory information."
                    )
            else:
                dispatcher.utter_message(text="I couldn’t find anyone matching the description.")
        else:
            dispatcher.utter_message(
                    text=f"Sorry, I didn't understand the ID. Can you please repeat it?."
                )

        return []

class ActionGetOtherTrajectories(Action):
    def name(self) -> Text:
        return "action_get_other_trajectories"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        with open('./files/results.txt', 'r') as file:
            data = json.load(file)
        people = data['people']

        if(tracker.get_slot("person") is not None or tracker.get_latest_entity_values('person') is not None):
            
            # Verifica se è stata trovata una persona corrispondente
            people = data['people']
            person_id = next(tracker.get_latest_entity_values('person'), None)
            if(person_id):
                person_id = int(person_id)
            else:
                person_id = tracker.get_slot("person")
                person_id = int(person_id)
            person = next((p for p in people if p['id'] == person_id),None)
            if(person):
                last_seen_info = person.get('trajectory', [])
                if len(last_seen_info) > 0:
                # Calcola il nuovo indice
                    
                    dispatcher.utter_message(
                        text=f"Let me check in my database. He should have passed closed to store {last_seen_info} some minutes ago ."
                        )

                else:
                    dispatcher.utter_message(
                        text=f"I found a match but I don’t have trajectory information on {person_id}."
                    )
        else:
            dispatcher.utter_message(
                text=f"Sorry, I didn't understand the ID. Can you please repeat it?."
            )

        return []
    
class ActionGetPeopleInTrajectories(Action):
    def name(self) -> Text:
        return "action_get_people_in_trajectories"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        with open('./files/results.txt', 'r') as file:
            data = json.load(file)
        people = data['people']

        if(tracker.get_slot("person") is not None or tracker.get_latest_entity_values('person') is not None):
            
            # Verifica se è stata trovata una persona corrispondente
            people = data['people']
            person_id = next(tracker.get_latest_entity_values('person'), None)
            if(person_id):
                person_id = int(person_id)
            else:
                person_id = tracker.get_slot("person")
                person_id = int(person_id)
            person = next((p for p in people if p['id'] == person_id),None)
            if(person):
                last_seen_info = person.get('trajectory', [])

                if len(last_seen_info) > 0:
                # Calcola il nuovo indice

                    dispatcher.utter_message(
                        text=f"Before I saw a person with that features close to the stores:  {last_seen_info}."
                        )

                else:
                    dispatcher.utter_message(
                        text=f"I found a match but I don’t have trajectory information on {person_id}."
                    )
        else:
            dispatcher.utter_message(
                text=f"Sorry, I didn't understand the ID. Can you please repeat it?."
            )

        return []