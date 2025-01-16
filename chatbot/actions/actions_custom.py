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
from rasa_sdk.forms import FormValidationAction

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

traj_dict_inv={
    "PINCO PALLINO": 1,
    "UNISA": 2,
    "COGNITIVE STORE": 3,
    "PEPPER'S CAFE": 4,
    "AV CAMERAS": 5,
    "DATA ANALYSIS INTIMO": 6,
    "NLP FOR KIDS": 7
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
        shop_name=next(tracker.get_latest_entity_values('shops'), None)
        txt=" "
        if(shop_name is not None):
            
            print(str(shop_name))
            if str(shop_name).upper() in traj_dict_inv:
                shop = traj_dict_inv[str(shop_name).upper()]
                filtered_people = [p for p in filtered_people if shop in p.get('trajectory')]
                txt = f" in {shop_name}."
            else:
                txt = f" but the {shop_name} is not in this mall!"
        
        # Conta il numero di persone filtrate
        count = len(filtered_people)
        if count > 0:
            dispatcher.utter_message(text=f"There are {count} people in the mall matching your criteria" + txt + "Do you want information about a specific ID?")
        else:
            dispatcher.utter_message(text="There are no people that match your criteria in the mall. Can i help ypu in some other way?")
        
        return [SlotSet("attribute", None)]



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
                dispatcher.utter_message(text=f"Information about the person with {person_id} id: {person}. You can ask me the positions of this ID to know the shops that he visited!")
            else:
                dispatcher.utter_message(text=f"No person found with ID {person_id} in the mall.")
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
        shop_name=next(tracker.get_latest_entity_values('shops'), None)
        if(shop_name is not None):
            
            print(str(shop_name))
            if str(shop_name).upper() in traj_dict_inv:
                shop = traj_dict_inv[str(shop_name).upper()]
                filtered_people = [p for p in filtered_people if shop in p.get('trajectory')]
                txt = f" in {shop_name}"
            else:
                txt = f" but the {shop_name} is not in this mall!"
        
        # Conta il numero di persone filtrate
        id_founded= None
        count = len(filtered_people)
        if count > 0:
            id_founded=[p.get('id') for p in filtered_people]
            dispatcher.utter_message(text=f"There are {count} people matching your criteria. And they are : {id_founded}, if you want information by one, specify the ID")
        else:
            dispatcher.utter_message(text="Sorry, there's no people match your criteria.")
        
        return [SlotSet("person", id_founded), SlotSet("attribute", None)]

class ClearActionAttribute(Action):
    def name(self) -> Text:
        return "clear_action_attribute"

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
                if len(filtered_people)>0:
                    if value == "hat":
                        filtered_people = [p for p in filtered_people if p.get('hat') == 'Yes']
                    elif value == "bag":
                        filtered_people = [p for p in filtered_people if p.get('bag') == 'Yes']
                    elif value == "male":
                        filtered_people = [p for p in filtered_people if p.get('gender') == 'Male']
                    elif value == "female":
                        filtered_people = [p for p in filtered_people if p.get('gender') == 'Female']
        # Conta il numero di persone filtrate
        id_founded= None
        if len(filtered_people) == 1:
            id_founded=[p.get('id') for p in filtered_people]
            dispatcher.utter_message(text=f"There is one person matching your criteria. And is : {id_founded}. Ask me the positions of this ID if you want.")
        if len(filtered_people) > 0:
            count= len(filtered_people)
            id_founded=[p.get('id') for p in filtered_people]
            dispatcher.utter_message(text=f"There are {count} people matching your criteria. And they are : {id_founded}, if you want information by one, specify the ID")
        else:
            dispatcher.utter_message(text="Sorry, there's no people match your criteria.")
        
        return [SlotSet("person", id_founded), SlotSet("attribute", None)]    

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

            if len(filtered_people) == 1:
                last_seen_info = filtered_people[0].get('trajectory', [])

                
                # Calcola il nuovo indice
                if len(last_seen_info)>0:  
                    last_seen_info = [traj_dict[p] for p in last_seen_info]
                    location = last_seen_info[-1]
                    dispatcher.utter_message(
                        text=f"Let me check in my database. Person with ID:{filtered_people[0].get('id')} should have passed close to store {location}."
                    )
                else:
                    dispatcher.utter_message(
                        text=f"Let me check in my database. I don't have location, i'm sorry."
                    )
                return [SlotSet("person", filtered_people[0].get('id')),SlotSet("attribute", None)]
            elif len(filtered_people)>0:
                messages = []
                init_message="Let me check in my database..."
                messages.append(init_message)
                len_message=f"there is {len(filtered_people)} match"
                messages.append(len_message)
                count = 0
                for p in filtered_people:
                    last_seen_info = p.get('trajectory', [])
                    last_seen_info = [traj_dict[loc] for loc in last_seen_info]
                    
                    if len(last_seen_info) > 0:
                        message = f"Person with ID:{p.get('id')} should have passed close to store {last_seen_info[-1]}."
                    else:
                        message = f"I don't have location for Person with ID:{p.get('id')}, I'm sorry."
                    
                    # Append the generated message to the messages list
                    messages.append(message)
                    
                    count += 1
                    if count >= 3:  # Exit after processing 3 people
                        message=f"these are the first three ids matched"
                        break
                    messages.append(message)
                            
                        
               

                # Join all messages into one string and send it
                final_message = "\n".join(messages)  # Join all messages with newline for separation
                dispatcher.utter_message(text=final_message)

        else:
            dispatcher.utter_message(
                    text=f"Sorry, i didn't found a match."
                )
        return[SlotSet("attribute", None)]


class ActionGetOtherTrajectories(Action):
    def name(self) -> Text:
        return "action_get_other_trajectories"

    def run(self, dispatcher: CollectingDispatcher,
            tracker: Tracker,
            domain: Dict[Text, Any]) -> List[Dict[Text, Any]]:

        with open('./files/results.txt', 'r') as file:
            data = json.load(file)
        people = data['people']

        if(tracker.get_slot("person") is not None or next(tracker.get_latest_entity_values('person'), None) is not None):
            
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
                    last_seen_info = [traj_dict[p] for p in last_seen_info]
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
    
class ActionComparison(Action):
        def name(self) -> Text:
            return "action_comparison"

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
            filtered_dict={}
            filtered_list=[]
            for attribute in attributes:
                if attribute['entity'] == "attribute" or attribute["entity"] == "shops":
                    value = attribute['value']
                    if value == "hat":
                        filtered_list = [p for p in filtered_people if p.get('hat') == 'Yes']
                    elif value == "bag":
                        filtered_list = [p for p in filtered_people if p.get('bag') == 'Yes']
                    elif value == "male":
                        filtered_list = [p for p in filtered_people if p.get('gender') == 'Male']
                    elif value == "female":
                        filtered_list = [p for p in filtered_people if p.get('gender') == 'Female']
                    elif attribute["entity"] == "shops":
                        if(value is not None):
                            if str(value).upper() in traj_dict_inv:
                                shop = traj_dict_inv[str(value).upper()]
                                filtered_list = [p for p in filtered_people if shop in p.get('trajectory')]
                    
                    filtered_dict[value]=filtered_list
            max= 0
            max_attribute= " "
            for key, value in filtered_dict.items():
                if(len(value) > max):
                    max= len(value)
                    max_attribute=key

            if max > 0:
                dispatcher.utter_message(text=f"There are more {max_attribute}, they are {max}" )
            else:
                dispatcher.utter_message(text="Can you repeat please?.")
            
            return [SlotSet("attribute", None)]